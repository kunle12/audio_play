#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstPlay
  {
    public:
      RosGstPlay()
      {
        GstPad *audiopad;

        std::string dst_type;
        std::string audio_format;
        int channels;
        int samplerate;
        int depth;

        ros::NodeHandle priNh( "~" );
        // The destination of the audio
        priNh.param<std::string>( "dst", dst_type, "alsasink" );
        priNh.param<std::string>( "format", audio_format, "pcm" );
        priNh.param( "channels", channels, 1 );
        priNh.param( "samplerate", samplerate, 96000 );
        priNh.param( "depth", depth, 16 );

        _pcm_format = (audio_format.compare( "pcm" ) == 0);
        _sub = _nh.subscribe( "/audio", 10, &RosGstPlay::onAudio, this );

        _loop = g_main_loop_new( NULL, false );

        _pipeline = gst_pipeline_new( "app_pipeline" );

        GstBus * audioBus = gst_pipeline_get_bus( GST_PIPELINE(_pipeline) );
        gst_bus_add_watch( audioBus, &RosGstPlay::audioBusMonitorCallback, this );
        g_object_unref( audioBus );

        _source = gst_element_factory_make( "appsrc", "app_source" );
        gst_bin_add( GST_BIN(_pipeline), _source );


        //_playbin = gst_element_factory_make("playbin2", "uri_play");
        //g_object_set( G_OBJECT(_playbin), "uri", "file:///home/test/test.mp3", NULL);
        if (dst_type == "alsasink") {
          _sink = gst_element_factory_make( "autoaudiosink", "sink" );
          if (_pcm_format) {
            gst_app_src_set_stream_type( GST_APP_SRC(_source), GST_APP_STREAM_TYPE_STREAM );
            GstCaps * audioCaps = gst_caps_new_simple( "audio/x-raw", "rate", G_TYPE_INT, samplerate,
                "channels", G_TYPE_INT, (gint)channels, "endianness", G_TYPE_INT,(gint)1234, "width", G_TYPE_INT,
                (gint)depth, "depth", G_TYPE_INT, (gint)depth,
                "signed", G_TYPE_BOOLEAN, TRUE, NULL );
            //GstCaps * audioCaps = gst_caps_from_string("audio/x-raw,format=S16LE,rate=96000,channels=1");
            gst_app_src_set_caps( GST_APP_SRC(_source), audioCaps );
            //g_object_set( G_OBJECT ( _source ), "is-live", TRUE, "min-percent", 50, "max-bytes",
            //    16384, "do-timestamp", TRUE, "block", TRUE, NULL );
            gst_bin_add( GST_BIN(_pipeline), _sink );

            if (!gst_element_link_many( _source, _sink, NULL )) {
              ROS_ERROR( "unable to link source with sink\n" );
            }
            gst_caps_unref( audioCaps );
          }
          else {
            _decoder = gst_element_factory_make( "decodebin", "decoder" );
            g_signal_connect( _decoder, "pad-added", G_CALLBACK(cb_newpad), this );
            gst_bin_add( GST_BIN(_pipeline), _decoder );
            gst_element_link( _source, _decoder );

            _audio = gst_bin_new( "audiobin" );
            _convert = gst_element_factory_make( "audioconvert", "convert" );
            audiopad = gst_element_get_static_pad( _convert, "sink" );
            gst_bin_add_many( GST_BIN(_audio), _convert, _sink, NULL );
            gst_element_link(_convert, _sink );
            gst_element_add_pad( _audio, gst_ghost_pad_new("sink", audiopad) );
            gst_object_unref( audiopad );

            gst_bin_add( GST_BIN(_pipeline), _audio );
          }
        }
        else {
          _sink = gst_element_factory_make( "filesink", "sink" );
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL );
          gst_bin_add( GST_BIN(_pipeline), _sink );
          gst_element_link( _source, _sink );
        }

        g_signal_connect( _source, "need-data", G_CALLBACK(cb_need_data), this );
        gst_element_set_state( GST_ELEMENT(_pipeline), GST_STATE_PLAYING );
        //gst_element_set_state(GST_ELEMENT(_playbin), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind( g_main_loop_run, _loop ) );

        _paused = false;
      }

    private:
      void onAudio( const audio_common_msgs::AudioDataConstPtr & msg )
      {
        if (_paused) {
          gst_element_set_state( GST_ELEMENT(_pipeline), GST_STATE_PLAYING );
          _paused = false;
        }
        GstBuffer *buffer = gst_buffer_new_and_alloc( msg->data.size() );
        gst_buffer_fill( buffer, 0, &msg->data[0], msg->data.size() );

        GstFlowReturn ret;

        ret = gst_app_src_push_buffer( GST_APP_SRC(_source), buffer );
        //printf( "buffer push %d\n", (int)ret );

        //g_signal_emit_by_name( _source, "push-buffer", buffer, &ret );
      }

      static gboolean audioBusMonitorCallback( GstBus * bus, GstMessage * message, gpointer data )
      {
        return ((RosGstPlay *)data)->audioMonitor( message );
      }

      gboolean audioMonitor( GstMessage * message )
      {
        switch (GST_MESSAGE_TYPE(message))
        {
          case GST_MESSAGE_ERROR:
          {
            gchar *debug;
            GError *err;

            gst_message_parse_error(message, &err, &debug);
            g_print("Error %s\n", err->message);
            g_error_free(err);
            g_free(debug);
            //isStreaming_ = false;
          }
          break;

          case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            /*
            if (isStreaming_) {
              printf( "reset the pipeline\n");
              gst_element_set_state( (GstElement *)audioPipeline_, GST_STATE_NULL );
              usleep( 1000 );
              gst_element_set_state( (GstElement *)audioPipeline_, GST_STATE_PLAYING );
            }
            */
            break;

          default:
          {
            g_print("got message %s\n",
                gst_message_type_get_name (GST_MESSAGE_TYPE (message)));
          }
            break;
        }

        return TRUE;
      }

      static void cb_newpad( GstElement * decodebin, GstPad * pad, gpointer data )
      {
        RosGstPlay *client = reinterpret_cast<RosGstPlay*>(data);

        GstCaps *caps;
        GstStructure *str;
        GstPad *audiopad;

        /* only link once */
        audiopad = gst_element_get_static_pad( client->_audio, "sink" );
        if (GST_PAD_IS_LINKED( audiopad )) {
          g_object_unref( audiopad );
          return;
        }

        /* check media type */
        caps = gst_pad_query_caps( pad, NULL );
        str = gst_caps_get_structure( caps, 0 );
        if (!g_strrstr( gst_structure_get_name (str), "audio" )) {
          gst_caps_unref( caps );
          gst_object_unref( audiopad );
          return;
        }

        gst_caps_unref( caps );

        /* link'n'play */
        gst_pad_link( pad, audiopad );

        g_object_unref( audiopad );
      }

      static void cb_need_data( GstElement * appsrc, guint unused_size, gpointer user_data )
      {
        ROS_WARN( "need-data signal emitted! Pausing the pipeline" );
        RosGstPlay * client = reinterpret_cast<RosGstPlay*>(user_data);
        gst_element_set_state( GST_ELEMENT(client->_pipeline), GST_STATE_PAUSED );
        client->_paused = true;
      }

      ros::NodeHandle _nh;
      ros::Subscriber _sub;
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_decoder, *_convert, *_audio;
      //GstElement *_playbin;
      GMainLoop *_loop;

      bool _paused;
      bool _pcm_format;
  };
}

int main (int argc, char **argv)
{
  ros::init( argc, argv, "audio_play" );
  gst_init( &argc, &argv );

  audio_transport::RosGstPlay client;

  ros::spin();
}
