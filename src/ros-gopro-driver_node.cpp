// ROS libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

// Include c++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

//https://stackoverflow.com/questions/10715170/receiving-rtsp-stream-using-ffmpeg-library

//FFmpeg
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
}


int main(int argc, char* argv[])
{
    //Init ROS
    ros::init(argc,argv,"gopro_video");

    //FFmpeg containers
    SwsContext *img_convert_ctx;
    AVFormatContext* format_ctx = avformat_alloc_context();
    AVCodecContext* codec_ctx = NULL;

    //Video stream index
    int video_stream_index;

    //Start FFmpeg
    av_register_all();
    avformat_network_init();

    //Open RTSP stream
    if(avformat_open_input(&format_ctx, "udp://10.5.5.9:8554", NULL, NULL) !=0)
    {
        return EXIT_FAILURE;
    }

    //Check stream info
    if(avformat_find_stream_info(format_ctx, NULL) < 0)
    {
        return EXIT_FAILURE;
    }

    //Search for the video stream
    for(int i=0; i < format_ctx->nb_streams; i++)
    {
        if(format_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
            video_stream_index = i;
    }

    //Create the package
    AVPacket packet;
    av_init_packet(&packet);

    //Open output file (DEL)
    AVFormatContext* output_ctx = avformat_alloc_context();

    //Stream container
    AVStream* stream = NULL;

    //Frame counter (DEL)
    int cnt = 0;

    //Start reading packets from stream and write them to file
    av_read_play(format_ctx);

    //Get the codec
    AVCodec* codec = NULL;
    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    //codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if(!codec)
    {
        exit(EXIT_FAILURE);
    }

    //Add this to allocate the context by codec
    codec_ctx = avcodec_alloc_context3(codec);

    //Set the context into the codec
    avcodec_get_context_defaults3(codec_ctx, codec);
    avcodec_copy_context(codec_ctx, format_ctx->streams[video_stream_index]->codec);

    //Create the stream for the output file
    std::ofstream output_file;

    //Open codec, exit if failure
    if(avcodec_open2(codec_ctx, codec, NULL) < 0)
        return EXIT_FAILURE;

    //Get the frame context to convert
    img_convert_ctx = sws_getContext(codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt, codec_ctx->width, codec_ctx->height, AV_PIX_FMT_RGB24, SWS_BICUBIC, NULL, NULL, NULL);

    //Get the original frame size in bytes
    int size = avpicture_get_size(AV_PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);

    //Create the input frame buffer
    uint8_t* picture_buffer = (uint8_t*)(av_malloc(size));

    //Allocate memory for the imput frames
    AVFrame* picture = av_frame_alloc();
    
    //Allocate memory for the output frames
    AVFrame* picture_rgb = av_frame_alloc();

    //Get the output frame size
    int size2 = avpicture_get_size(AV_PIX_FMT_RGB24, codec_ctx->width, codec_ctx->height);

    //Create the output frame buffer
    uint8_t* picture_buffer_2 = (uint8_t*)(av_malloc(size2));

    //Fill out the containers
    avpicture_fill((AVPicture*) picture, picture_buffer, AV_PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
    avpicture_fill((AVPicture*) picture_rgb, picture_buffer_2, AV_PIX_FMT_RGB24, codec_ctx->width, codec_ctx->height);

    //Start reading the frames
    while(av_read_frame(format_ctx, &packet) >= 0)
    {
        std::cout<<"[1] Frame: "<<cnt<<std::endl;

        //Identify if this is a video stream
        if(packet.stream_index == video_stream_index)
        {
            std::cout<<"[2] Is Video"<<std::endl;
            
            //Create stream in file
            if(stream == NULL)
            {
                std::cout<<"[3] Create stream"<<std::endl;

                //Set the stream
                stream = avformat_new_stream(output_ctx, format_ctx->streams[video_stream_index]->codec->codec);
                //Copy the context into the stream
                avcodec_copy_context(stream->codec, format_ctx->streams[video_stream_index]->codec);

                //Set the sample aspect ratio
                stream->sample_aspect_ratio = format_ctx->streams[video_stream_index]->codec->sample_aspect_ratio;
            }

            //Result of decoding
            int check = 0;

            //Set packet stream
            packet.stream_index = stream->id;
            
            std::cout<<"[4] decoding" << std::endl;

            //Decode video
            int result = avcodec_decode_video2(codec_ctx, picture, &check, &packet);
            std::cout<< "Bytes decoded " << result << " check " << check << std::endl; 

            //Let some frames to pass (0 to proceed immediately)
            /*if(cnt)
            {

                std::cout<<"Saving frame..."<<std::endl;

                //Scale image
                sws_scale(img_convert_ctx, picture->data, picture->linesize, 0, codec_ctx->height, picture_rgb->data, picture_rgb->linesize);

                //Create the files
                std::stringstream file_name;
                file_name << "test" << cnt << ".ppm";
                //file_name << "test.ppm";
                
                //Create and open the file
                output_file.open(file_name.str().c_str());

                //Insert PPM header
                output_file << "P3 " <<codec_ctx->width<< " " << codec_ctx->height<< " 255\n";

                //Fill out the file with data
                for (int y=0; y<codec_ctx->height;y++)
                {
                    for(int x=0; x<codec_ctx->width * 3; x++)
                    {
                        output_file<<(int)(picture_rgb->data[0]+y*picture_rgb->linesize[0])[x] << " ";
                    }
                }

                //Close file
                output_file.close();
            }*/
            
            //Update counter
            cnt++;
        }

        //Free the packet
        av_free_packet(&packet);

        //Re-init the packet for the upcoming frame
        av_init_packet(&packet);
    }

    //Release resources
    av_free(picture);
    av_free(picture_rgb);
    av_free(picture_buffer);
    av_free(picture_buffer_2);

    //Stop communications
    av_read_pause(format_ctx);
    avio_close(output_ctx->pb);
    
    //Release output context
    avformat_free_context(output_ctx);

    return EXIT_SUCCESS;
}
