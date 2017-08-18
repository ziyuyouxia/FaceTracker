// g++ find_face_center.cpp -o ffc `pkg-config opencv --cflags --libs`
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <stdio.h>
#include <math.h>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h> // for close()

using namespace cv;

/** Function Headers */
Point_<double> find_centroid(Mat& frame);
int i2c_open();
void i2c_close();
void i2c_check(int);
void i2c_send_face_pos(Point_<double>&);

/** Global variables */
String face_cascade_name = "haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
double FRAME_HEIGHT = 480.0;
double FRAME_WIDTH = 640.0;
int i2c_device;
const unsigned int PI_ADDR = 0x04 , FACE_DETECT_REG = 0x00 , FACE_POS_REG = 0x01; //i2c stuff
int face_detected = 0;


int main(void)
{
    int i2cChk = i2c_open();
    if(i2cChk < 0) {
        //error message in i2c_open();
        return -1;
    }

    if (!face_cascade.load(face_cascade_name))
    {
        printf("--(!)Error loading face cascade\n");
        return -1;
    };

    VideoCapture capture;
    Mat frame;

    capture.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    //-- 2. Read the video stream
    capture.open(-1);
    if (!capture.isOpened())
    {
        printf("Error opening video capture\nHave you run sudo modprobe bcm2835-v4l2?\n");
        return -1;
    }

    while (capture.read(frame))
    {
        if (frame.empty())
        {
            printf("No captured frame - exiting");
            break;
        }

        //-- 3. Apply the classifier to the frame, return relative height and width
        Point_<double> center = find_centroid(frame);
    }
    return 0;
}

Point_<double> find_centroid(Mat &frame)
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
    std::size_t num_faces = faces.size();

    if (num_faces > 0)
    {
        //there might be more than one face: if so, select the one with the biggest area
        int face_area_max = 0 , face_index = 0;
        for (std::size_t i = 0; i < num_faces; i++)
        {
            int face_area = faces[i].width * faces[i].height;
            if (face_area > face_area_max)
            {
                face_index = i;
            }
        }

        //get the centre co-ordinates of the face
        Point center(faces[face_index].x + faces[face_index].width / 2,
                     faces[face_index].y + faces[face_index].height / 2);

        //translate centre point into a relative 0:1 value
        Point_<double> center_relative((double)(center.x) * 1.0 / FRAME_WIDTH,
                                       (double)(center.y) * 1.0 / FRAME_HEIGHT); // note use Point_ template, Point is int only

        //if we previously hadn't detected a face, let the Teensy know we have now, then send the position
        if (face_detected == 0)
        {
            face_detected = 1;
            int write_chk = i2c_smbus_write_byte_data(i2c_device, 0, face_detected);
            i2c_check(write_chk);
        }
        i2c_send_face_pos(center_relative);

        
        //print the result
        std::cout << round((center_relative.x - 0.5)*100)/100 \
        << "  " << round((center_relative.y - 0.5)*100)/100  << std::endl;
    }
    else
    //no face :(
    {
        std::cout << "no face detected." << std::endl;
        if (face_detected == 1)
        {
            //we had a face but don't anymore, update the Teensy
            face_detected = 0;
            int write_chk = i2c_smbus_write_byte_data(i2c_device, FACE_DETECT_REG, face_detected);
            i2c_check(write_chk);
        }
    }
}

int i2c_open()
{
    //open i2c bus
    i2c_device = open("/dev/i2c-1", O_RDWR);
    if (i2c_device < 0)
    {
        std::cout << "couldn't find I2C device" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "I2C Bus opened" << std::endl;
    }

    if (ioctl(i2c_device, I2C_SLAVE, PI_ADDR) < 0)
    {
        std::cout << "error setting I2C address" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "I2C address set" << std::endl;
    }

    return 1;
}

void i2c_close()
{
    close(i2c_device);
    std::cout << "i2c closed" << std::endl;
}

void i2c_check(int write_chk)
{
    if (write_chk < 0)
    {
        std::cout << "Error sending face_detected message over I2C" << std::endl;
        i2c_close();
        int i2c_chk = i2c_open();
    }
}

void i2c_send_face_pos(Point_<double>& center_relative)
{
    int x16 = (int)(center_relative.x * 65535.0);
    int y16 = (int)(center_relative.y * 65535.0);
    uint8_t i2c_data[4] = {x16 >> 8, x16 & 255, y16 >> 8, y16 & 255};
    uint8_t *coords[4] = {&i2c_data[0], &i2c_data[1], &i2c_data[2], &i2c_data[3]};

    int write_chk = i2c_smbus_write_i2c_block_data(i2c_device, FACE_POS_REG, 4, *coords);
    i2c_check(write_chk);
}