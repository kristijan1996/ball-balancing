#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include "SerialPort.hpp"

using namespace cv;
using namespace std;

// Rows in between which original camera frame is cropped
// which should only contain the rail and the ball, in order
// to minimize interference from other object camera sees
int lowFrameCutoff = 185;
int highFrameCutoff = 295;

// Define range of HSV values in which the white ball is best
// extracted from the rest of the frame
Scalar lowBoundHSV { 0, 0, 255 };
Scalar highBoundHSV { 255, 255, 255 };

/** 
    Finds contour with largest area among contours in vector container

    @param[in] contours - reference to vector<vector<Point>> representing
                          contours retreived from findContours OpenCV method

    @returns index of largest contour in container
*/
int findLargestContour(vector<vector<Point>>& contours)
{
    int index = 0;
    double largestArea = 0;

    for (auto i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours.at(i));
        if (area > largestArea)
        {
            largestArea = area;
            index = i;
        }
    }

    return index;
}

/** 
    Function which sends passed value val over serial
    to MCU. Value is formatted in such a way that three
    digit val is sent directly, while two and one digit
    vals are prepended with 'A' and 'AA', because zeroes
    are interpreted as end of string.

    @param[in] serial - pointer to SerialPort instance which is
                        connected to proper COM port on which
                        MCU is connected
    @param[in] value - integer value to be sent over UART
*/
void sendValueOverUART(SerialPort* serial, int value)
{
    string s = to_string(value);
    if (s.length() == 1)
        s = "AA" + s;
    else if (s.length() == 2)
        s = "A" + s;

    s = s + "\r\n";
    serial->writeSerialPort(s.c_str(), 5);
}

/**
    Aux function used to set the camera properly so it only
    sees the rail, in order to minimize distractions from 
    reflections around it
*/
int calibrate()
{
    // Instanciate camera object
    VideoCapture cap;

    try
    {
        if (!cap.open(0))
        {
            cout << "Unable to open the camera!" << endl;
            return -1;
        }

        // Placeholder matrices to hold the original frame grabbed from the camera
        // and the cropped version of the frame showing just the rail and the ball
        Mat frame;
        Mat cropped;

        while (true)
        {
            if (!cap.read(frame)) 
            {
                cout << "Problems connecting to camera!" << endl;
                continue;
            }

            cropped = frame(Range(lowFrameCutoff, highFrameCutoff), Range::all());
            imshow("Calibration", cropped);

            if (waitKey(1) == 13)
                break;
        }

        cap.release();
        destroyAllWindows();
    }
    catch (const exception& e)
    {
        // Catch anything thrown within try block that derives from std::exception
        cout << "Exception occured!" << endl;
        cout << e.what();
        cap.release();
        cv::destroyAllWindows();
    }

    return 0;
}

/** 
    Grabs frame from camera and tries to detect circles
    in it. Once a circle has been detected, horizontal
    pixel value of its center is sent via UART to MCU.
    Middle point of the rail is considered to be aroud 320th
    pixel.
    Hint:
    ---------> x
    |
    |
    |
    y
    
*/
int run()
{
    // Instanciate camera and serial port objects
    VideoCapture cap;
    SerialPort* serial = new SerialPort("\\\\.\\COM5");

    try
    {
        if (!cap.open(0)) {
            cout << "Unable to open the camera!" << endl;
            return -1;
        }

        if (!serial->isConnected())
        {
            cout << "Serial not connected!" << endl;
            return -1;
        }

        // Placeholder for main frame grabbed from the camera
        Mat frame;
        // Frame is cropped to extract just the ball and the rail
        Mat cropped;
        // Placeholder for cropped frame which goes through various
        // transformations
        Mat processed;
        // Array of array of points -- that is - an array of contours
        vector<vector<Point>> contours;
        // Largest contour from the above array
        vector<Point> largestContour;
        // Not used, but needed for OpenCV function
        vector<Vec4i> hierarchy;
        // Placeholders for center and radius of the detected circle/ball
        Point2f center; float radius;

        // Main loop which grabs frame from camera, processes it,
        // finds ball and its center and sends x coordinate via UART
        while (true)
        {
            if (!cap.read(frame)) {
                cout << "Problems connecting to camera!" << endl;
                continue;
            }

            // Crop the grabbed frame to extract just the region with ball and rail
            cropped = frame(Range(lowFrameCutoff, highFrameCutoff), Range::all());
            // Blur it
            GaussianBlur(cropped, processed, Size(11, 11), 0);
            // Transform it to HSV color space in order to easily detect white ball
            cvtColor(processed, processed, COLOR_BGR2HSV);
            // White ball is best detected if thresholded within these boundaries
            inRange(processed, lowBoundHSV, highBoundHSV, processed);
            // Erode and dilate to fill the small gaps in the shape
            erode(processed, processed, Mat());
            dilate(processed, processed, Mat());

            // Find all contours in black and white image, where ball should be the
            // largest and most distinct shape, apart from the small disturbances coming
            // from reflections
            findContours(processed, contours, hierarchy, RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0, 0));

            if (contours.size() > 0)
            {
                // Find the largest contour among contours, which should be the ball we are looking for
                int index = findLargestContour(contours);
                largestContour = contours.at(index);

                // Enclose it with a circle
                minEnclosingCircle(largestContour, center, radius);
                
                // To make sure it is not a smudge, keep the radius in some range
                if (radius > 30 && radius < 100)
                {
                    // Draw the circle and its center on the original frame
                    circle(frame, Point(center.x, center.y + lowFrameCutoff), (int)radius, Scalar(0, 255, 0), 2);
                    circle(frame, Point(center.x, center.y + lowFrameCutoff), 5, Scalar(0, 0, 255), -1);

                    // Send the x coordinate of center via UART to MCU to process
                    sendValueOverUART(serial, center.x);

                    // Display the detected ball and the distance in pixels from center of the rail
                    int err = center.x - 322;
                    putText(frame, "Error: " + to_string(err), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
                }
            }
            else
            {
                cout << "Ball not detected!" << endl;
                sendValueOverUART(serial, 322);
            }

            // Show frame
            imshow("Camera", frame);

            if (waitKey(1) == 13)
                break;
        }

        cap.release();
        cv::destroyAllWindows();
        serial->closeSerial();

    }
    catch (const exception & e)
    {
        // Catch anything thrown within try block that derives from std::exception
        cout << "Exception occured!" << endl;
        cout << e.what();
        cap.release();
        cv::destroyAllWindows();
        serial->closeSerial();
    }

    return 0;
}


int main()
{
    int c;
    // Display nice main menu for user to pick
    while (true)
    {
        cout << "\nPick an option:\n";
        cout << "1. Calibrate camera\n";
        cout << "2. Run ball tracking\n";
        cout << "3. Exit\n";
        cout << "Choice: ";
        cin >> c;
        cout << endl;

        switch (c)
        {
        case 1:
            calibrate();
            break;

        case 2:
            run();
            break;
        
        default:
            return 0;
        }
    }
}
