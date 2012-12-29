using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using Microsoft.Kinect;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.ComponentModel;
using System.Security;
using System.Security.Permissions;
using System.IO;

namespace Kinecursor
{
    /// <summary>
    /// Kinecursor v1.1 by TangoChen
    /// Twitter / Weibo(微博): @TangoChen
    /// Youtube channel: http://www.youtube.com/tan9ochen
    /// Subscribe to my Youtube channel for more Kinect projects.. :)
    /// Blog: http://www.TangoChen.com
    
    /// This is based on some samples in the Toolkit..
    /// Most of the summaries below are not written by TangoChen..
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private short[] depthPixels;

        /// <summary>
        /// Intermediate storage for the depth data converted to color
        /// </summary>
        private byte[] colorPixels;

        const int MaxDepthDistance = 4095; // max value returned
        const int MinDepthDistance = 800; // min value returned
        const int MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;

        int screenWidth = (int)SystemParameters.PrimaryScreenWidth;
        int screenHeight = (int)SystemParameters.PrimaryScreenHeight;

        bool handReady = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {

            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);

                this.sensor.SkeletonStream.Enable();

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new short[this.sensor.DepthStream.FramePixelDataLength];

                // Allocate space to put the color pixels we'll create
                this.colorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.imgDepth.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;

                this.sensor.DepthStream.Range = DepthRange.Default;

                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                MessageBox.Show("No Kinect Found.. :(");
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyPixelDataTo(this.depthPixels);

                    colorPixels = GenerateColoredBytes(depthFrame);

                    // Write the pixel data into our bitmap
                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            if (skeletons.Length != 0)
            {
                foreach (Skeleton skel in skeletons)
                {

                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        Joint jointHandLeft = skel.Joints[JointType.HandLeft];
                        Joint jointHandRight = skel.Joints[JointType.HandRight];
                        Joint jointShoulderCenter = skel.Joints[JointType.ShoulderCenter];

                        //tbInfo.Text = (jointHandRight.Position.Z - jointShoulderCenter.Position.Z).ToString();
                        //tbInfo.Text = (jointHandLeft.Position.Z - jointShoulderCenter.Position.Z).ToString();

                        if (jointHandRight.Position.Z - jointShoulderCenter.Position.Z < -0.3)
                        {
                            float x = jointHandRight.Position.X - jointShoulderCenter.Position.X;//hX - sX;
                            float y = jointShoulderCenter.Position.Y - jointHandRight.Position.Y;//sY - hY;
                            handReady = true;
                            SetCursorPos((int)((x + 0.05) / 0.35 * screenWidth), (int)(y / 0.35 * screenHeight));
                        }
                        else if (jointHandLeft.Position.Z - jointShoulderCenter.Position.Z < -0.3)
                        {

                            float x = jointHandLeft.Position.X - jointShoulderCenter.Position.X;//hX - sX;
                            float y = jointShoulderCenter.Position.Y - jointHandLeft.Position.Y;//sY - hY;
                            handReady = true;
                            SetCursorPos((int)((x + 0.3) / 0.35 * screenWidth), (int)(y / 0.35 * screenHeight));
                        }
                        else
                        {
                            handReady = false;
                        }

                        tbInfo.Text = "Fist: "+isMakingAFist.ToString();
                    }
                }
            }

        }

        bool isMouseDown = false;
        int handDepthFirst = 0;

        //hardcoded locations to Blue, Green, Red (BGR) index positions       
        const int BlueIndex = 0;
        const int GreenIndex = 1;
        const int RedIndex = 2;

        int depthX = 0;
        bool isMakingAFist = false;
        int lastDepthClosest = MaxDepthDistance;

        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            //get the raw data from kinect with the depth for every pixel
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            //use depthFrame to create the image to display on-screen
            //depthFrame contains color information for all pixels in image
            //Height x Width x 4 (Red, Green, Blue, empty byte)
            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];


            //Bgr32  - Blue, Green, Red, empty byte
            //Bgra32 - Blue, Green, Red, transparency 
            //You must set transparency for Bgra as .NET defaults a byte to 0 = fully transparent

            int depthClosest = MaxDepthDistance;

            List<int> lstHandTop= new List<int>();
            
            //loop through all distances
            //pick a RGB color based on distance
            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                //get the player (requires skeleton tracking enabled for values)
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                //gets the depth value
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                
                if (player > 0)
                {

                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 255;

                    if (depth < depthClosest)
                    {
                        depthClosest = depth;
                    }

                    //Do Finger Tracking
                    if (handReady)
                    {
                        if (depth - lastDepthClosest < 60)
                        {
                            pixels[colorIndex + BlueIndex] = 0;
                            pixels[colorIndex + GreenIndex] = 255;
                            pixels[colorIndex + RedIndex] = 255;

                            depthX = GetX(depthIndex);
                            //depthY = GetY(depthIndex);

                            if (!lstHandTop.Contains(depthX))
                            {
                                if (lstHandTop.Count == 0)
                                {
                                    handDepthFirst = depth;
                                }
                                lstHandTop.Add(depthX);

                                pixels[colorIndex + BlueIndex] = 0;
                                pixels[colorIndex + GreenIndex] = 255;
                                pixels[colorIndex + RedIndex] = 0;
                            }

                        }
                    }

                        
                }
                else
                {
                    byte intensity = CalculateIntensityFromDepth(depth);
                    pixels[colorIndex + BlueIndex] = intensity;
                    pixels[colorIndex + GreenIndex] = intensity;
                    pixels[colorIndex + RedIndex] = intensity;

                    //The last point...
                    if (IsInOnePoint(depthIndex, 319, 239))
                    {
                        lastDepthClosest = depthClosest;

                        int widthSeparator = (MaxDepthDistance - handDepthFirst) / 65-20;
                        
                        /*
                        tbInfo.Text = lstHandTop.Count.ToString() + "," + widthSeparator.ToString() + "," +
                                      (lstHandTop.Count > widthSeparator ? "True" : "False") + "," + handDepthFirst.ToString();
                        */

                        isMakingAFist = lstHandTop.Count > widthSeparator;

                        if (isMakingAFist)
                        {
                            if (!isMouseDown)
                            {
                                MouseLeftDown();
                                isMouseDown = true;
                            }
                        }
                        else
                        {
                            if (isMouseDown)
                            {
                                MouseLeftUp();
                                isMouseDown = false;
                            }
                        }
                    }
                }
                
            }

            return pixels;
        }

        private static byte CalculateIntensityFromDepth(int distance)
        {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0)
                / (MaxDepthDistanceOffset)));
        }

        public int GetX(int depth)
        {
            return (depth + 1) % 320;
        }

        //public int GetY(int depth)
        //{
        //    return (depth + 1) / 320;
        //}

        public bool IsInOnePoint(int depth, int x, int y)
        {
            if ((depth + 1) % 320 == x && (depth + 1) / 320 == y)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        #region Mouse Controll

        //Move Mouse
        //SetCursorPos(10, 10);

        //public void MouseLeftClick()
        //{
        //    mouse_event(MouseEventFlag.LeftDown | MouseEventFlag.Absolute, 0, 0, 0, UIntPtr.Zero);
        //    //Thread.Sleep(50);
        //    mouse_event(MouseEventFlag.LeftUp | MouseEventFlag.Absolute, 0, 0, 0, UIntPtr.Zero);
        //    //mouse_event(MouseEventFlag.LeftDown | MouseEventFlag.LeftUp | MouseEventFlag.Absolute, 0, 0, 0, UIntPtr.Zero);
        //}

        public void MouseLeftDown()
        {
            mouse_event(MouseEventFlag.LeftDown, 0, 0, 0, UIntPtr.Zero);
        }
        public void MouseLeftUp()
        {
            mouse_event(MouseEventFlag.LeftUp, 0, 0, 0, UIntPtr.Zero);
        }

        [DllImport("user32.dll")]
        static extern bool SetCursorPos(int X, int Y);
        [DllImport("user32.dll")]
        static extern void mouse_event(MouseEventFlag flags, int dx, int dy, uint data, UIntPtr extraInfo);
        [Flags]
        enum MouseEventFlag : uint
        {
            Move = 0x0001,
            LeftDown = 0x0002,
            LeftUp = 0x0004,
            RightDown = 0x0008,
            RightUp = 0x0010,
            MiddleDown = 0x0020,
            MiddleUp = 0x0040,
            XDown = 0x0080,
            XUp = 0x0100,
            Wheel = 0x0800,
            VirtualDesk = 0x4000,
            Absolute = 0x8000
        }
        #endregion



    }
}
