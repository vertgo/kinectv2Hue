//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace BodyBasicsWPF
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        ////////stuff from depth
        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int cbytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap bitmap = null;

        /// <summary>
        /// Intermediate storage for receiving frame data from the sensor
        /// </summary>
        private ushort[] frameData = null;

        /// <summary>
        /// Intermediate storage for receiving frame data from the sensor
        /// </summary>
        private byte[] pixels = null;

        private int curSelectedLight = -1;

        ////


        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        private const float SelectAngleMax = 0.3f;

        /// <summary>
        /// Radius of drawn light circles
        /// </summary>
        private const double LightSize = 10;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        private readonly Brush lightBrush = Brushes.White;
        private readonly Brush lightPointedBrush = Brushes.Yellow;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        //private CameraSpacePoint light1 = new CameraSpacePoint();

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        private ImageSource colorImageSource;


        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private MultiSourceFrameReader reader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        private CameraSpacePoint[] lightPositions = new CameraSpacePoint[3];

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// The time of the first frame received
        /// </summary>
        private long startTime = 0;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Next time to update FPS/frame time status
        /// </summary>
        private DateTime nextStatusUpdate = DateTime.MinValue;
        private DateTime lastSelectedTime = DateTime.MinValue;

        /// <summary>
        /// Number of frames since last FPS/frame time status
        /// </summary>
        private uint framesSinceUpdate = 0;

        /// <summary>
        /// Timer for FPS calculation
        /// </summary>
        private Stopwatch stopwatch = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // create a stopwatch for FPS calculation
            this.stopwatch = new Stopwatch();

            // for Alpha, one sensor is supported
            this.kinectSensor = KinectSensor.Default;


            //initialize hard coded light
            CameraSpacePoint light0 = new CameraSpacePoint();
            CameraSpacePoint light1 = new CameraSpacePoint();
            CameraSpacePoint light2 = new CameraSpacePoint();

            light0.X = -1.302601f;
            light0.Y = 0.653169f;
            light0.Z = 2.312274f;


            light1.X = -0.6725083f;
            light1.Y = -0.09831079f;
            light1.Z = 2.45966f;

            light2.X = -0.01340344f;
            light2.Y = 0.5311673f;
            light2.Z = 2.678232f;

            lightPositions[0] = light0;
            lightPositions[1] = light1;
            lightPositions[2] = light2;


            //lightPositions[ 0 ] = 

            if (this.kinectSensor != null)
            {
                // get the coordinate mapper
                this.coordinateMapper = this.kinectSensor.CoordinateMapper;
                
                // open the sensor
                this.kinectSensor.Open();

                // get the depth (display) extents
                FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
                this.displayWidth = frameDescription.Width;
                this.displayHeight = frameDescription.Height;
                
                FrameDescription depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;


                this.frameData = new ushort[frameDescription.Width * frameDescription.Height];
                this.pixels = new byte[frameDescription.Width * frameDescription.Height * this.cbytesPerPixel];

                // create the bitmap to display
                this.bitmap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);


                this.bodies = new Body[this.kinectSensor.BodyFrameSource.BodyCount];

                // open the reader for the body frames
                this.reader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.Body);

                // set the status text
                this.StatusText = Properties.Resources.InitializingStatusTextFormat;
            }
            else
            {
                // on failure, set the status text
                this.StatusText = Properties.Resources.NoSensorStatusText;
            }

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();


        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        public ImageSource ColorImageSource
        {
            get
            {
                return this.bitmap;
            }
        }
        
        /// <summary>
        /// return a vector that extends from csp1 to csp2
        /// </summary>
        public CameraSpacePoint difference(CameraSpacePoint csp1, CameraSpacePoint csp2)
        {
            CameraSpacePoint retPoint = new CameraSpacePoint();
            retPoint.X = csp2.X - csp1.X;
            retPoint.Y = csp2.Y - csp1.Y;
            retPoint.Z = csp2.Z - csp1.Z;
            return (retPoint);
        }

        
        /// <summary>
        /// return a the dot product between csp1->csp2 and csp1-csp3
        /// </summary>
        public float dotProduct( CameraSpacePoint vec1, CameraSpacePoint vec2 )//CameraSpacePoint csp1, CameraSpacePoint csp2, CameraSpacePoint csp3)
        {
            return (vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z);
        }

        public float magnitude(CameraSpacePoint csp)
        {
            return ( (float) Math.Sqrt( Math.Pow(csp.X, 2) + Math.Pow(csp.Y, 2) + Math.Pow(csp.Z, 2 ) ) );
        }

        public float angleBetweenVecs(CameraSpacePoint csp1, CameraSpacePoint csp2, CameraSpacePoint csp3)
        {
            CameraSpacePoint vec1 = difference(csp1, csp2);
            CameraSpacePoint vec2 = difference(csp1, csp3);

            return ( (float) Math.Acos( dotProduct(vec1, vec2)/ ( magnitude(vec1) * magnitude( vec2) ) ) );
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.reader != null)
            {
                this.reader.MultiSourceFrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.reader != null)
            {
                // BodyFrameReder is IDisposable
                this.reader.Dispose();
                this.reader = null;
            }

            // Body is IDisposable
            if (this.bodies != null)
            {
                foreach (Body body in this.bodies)
                {
                    if (body != null)
                    {
                        body.Dispose();
                    }
                }
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        private int getSelectedLight(CameraSpacePoint inShoulder, CameraSpacePoint inHand)
        {
            float lowestAngle = 300;
            int curClosestLight = -1;
            for (int i = 0; i < lightPositions.Length; i++) //hardcoded
            {
                float angleToLight = angleBetweenVecs(inShoulder, inHand, lightPositions[ i ] );
                if (angleToLight < SelectAngleMax && angleToLight < lowestAngle)
                {
                    lowestAngle = angleToLight;
                    curClosestLight = i;
                }
            }

            return curClosestLight;
        }

        private void deselectLight()
        {
            curSelectedLight = 0;
            lastSelectedTime = DateTime.Now;
        }
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            //System.Diagnostics.Debug.Write("Frame Arrived");
            MultiSourceFrameReference frameReference = e.FrameReference;

            MultiSourceFrame multiSourceFrame = null;
            //DepthFrame depthFrame = null;
            DepthFrame depthFrame = null;
            BodyFrame bodyFrame = null;

            

            try
            {
                multiSourceFrame = frameReference.AcquireFrame();

                if (multiSourceFrame != null)
                {
                    BodyFrameReference bodyFrameReference = multiSourceFrame.BodyFrameReference;
                    DepthFrameReference depthFrameReference = multiSourceFrame.DepthFrameReference;

                    if (this.startTime == 0)
                    {
                        this.startTime = bodyFrameReference.RelativeTime;
                    }

                    depthFrame = depthFrameReference.AcquireFrame();
                    bodyFrame = bodyFrameReference.AcquireFrame();

                    // BodyFrame is IDisposable
                    if ( (depthFrame != null) && (bodyFrame!= null))                        
                    {
                        this.framesSinceUpdate++;


                        FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                        //FrameDescription bodyFrameDescription = bodyFrame.FrameDe;
                        //no framedescription for the body frame i guess

                        // update status unless last message is sticky for a while
                        if (DateTime.Now >= this.nextStatusUpdate)
                        {
                            // calcuate fps based on last frame received
                            double fps = 0.0;

                            if (this.stopwatch.IsRunning)
                            {
                                this.stopwatch.Stop();
                                fps = this.framesSinceUpdate / this.stopwatch.Elapsed.TotalSeconds;
                                this.stopwatch.Reset();
                            }

                            this.nextStatusUpdate = DateTime.Now + TimeSpan.FromSeconds(1);
                            

                        }

                        if (!this.stopwatch.IsRunning)
                        {
                            this.framesSinceUpdate = 0;
                            this.stopwatch.Start();
                        }


                        //////depth stuff
                        //depthFrameDescription.System.Diagnostics.Debug.Write("color stuff? descwidth:" + depthFrameDescription.Width + ", bitmapWidth:" + this.bitmap.PixelWidth);

                        if (((depthFrameDescription.Width * depthFrameDescription.Height) == this.frameData.Length) &&
                            (depthFrameDescription.Width == this.bitmap.PixelWidth) && (depthFrameDescription.Height == this.bitmap.PixelHeight))
                        {
                            // Copy the pixel data from the image to a temporary array
                            depthFrame.CopyFrameDataToArray(this.frameData);

                            // Get the min and max reliable depth for the current frame
                            ushort minDepth = depthFrame.DepthMinReliableDistance;
                            ushort maxDepth = depthFrame.DepthMaxReliableDistance;

                            // Convert the depth to RGB
                            int colorPixelIndex = 0;
                            for (int i = 0; i < this.frameData.Length; ++i)
                            {
                                // Get the depth for this pixel
                                ushort depth = this.frameData[i];

                                // To convert to a byte, we're discarding the most-significant
                                // rather than least-significant bits.
                                // We're preserving detail, although the intensity will "wrap."
                                // Values outside the reliable depth range are mapped to 0 (black).
                                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                                // Write out blue byte
                                this.pixels[colorPixelIndex++] = intensity;

                                // Write out green byte
                                this.pixels[colorPixelIndex++] = intensity;

                                // Write out red byte                        
                                this.pixels[colorPixelIndex++] = intensity;

                                // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                                // If we were outputting BGRA, we would write alpha here.
                                ++colorPixelIndex;
                            }

                            this.bitmap.WritePixels(
                                new Int32Rect(0, 0, depthFrameDescription.Width, depthFrameDescription.Height),
                                this.pixels,
                                depthFrameDescription.Width * this.cbytesPerPixel,
                                0);
                        }

                        ///end depth stuff
                        

                        using (DrawingContext dc = this.drawingGroup.Open())
                        {
                            // Draw a transparent background to set the render size
                            //dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                            dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
  
                            // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                            // As long as those body objects are not disposed and not set to null in the array,
                            // those body objects will be re-used.
                            bodyFrame.GetAndRefreshBodyData(this.bodies);
                            bool firstFound = false;

                            //drawing lights

                            bool light1PointedAt = false;
                            foreach (Body body in this.bodies)
                            {
                                if (body.IsTracked)
                                {
                                    
                                    //this.DrawClippedEdges(body, dc);

                                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                    //make sure it's tracked
                                    //joints[JointType.Head].TrackingState == TrackingState.NotTracked
                                    // convert the joint points to depth (display) space
                                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                                    foreach (JointType jointType in joints.Keys)
                                    {
                                        DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(joints[jointType].Position);
                                        jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                    }

                                    if (!firstFound)
                                    {
                                        Joint rightShoulderJoint = joints[JointType.ShoulderRight];
                                        Joint rightHandJoint = joints[JointType.HandRight];


                                        CameraSpacePoint rhp = rightHandJoint.Position;
                                        CameraSpacePoint rsp = rightShoulderJoint.Position;


                                        if (rightShoulderJoint.TrackingState == TrackingState.Tracked &&
                                            rightHandJoint.TrackingState == TrackingState.Tracked)
                                        {
                                            //System.Diagnostics.Debug.Write("Right Shoulder");

                                           
                                            //System.Diagnostics.Debug.Write(shoulderRight.X + "," + shoulderRight.Y + "," + shoulderRight.Z);

                                            System.Diagnostics.Debug.Write("Right hand:");


                                            System.Diagnostics.Debug.WriteLine(rhp.X + "," + rhp.Y + "," + rhp.Z + "!!!");

                                            //System.Diagnostics.Debug.Write("DONE\n");

                                            //float angleToLight = angleBetweenVecs(rsp, rhp, light1);

                                            //System.Diagnostics.Debug.WriteLine( "Angle between hand and light:" + angleToLight);

                                            int newSelected = getSelectedLight(rsp, rhp);

                                            if (newSelected > -1 )
                                            {
                                                curSelectedLight = newSelected;
                                                lastSelectedTime = DateTime.Now;

                                            }
                                            else
                                            {
                                                //no light was selected and it's been more than 10 seconds from the last time you pointed at a light
                                                if (curSelectedLight < -1 && DateTime.Now >= lastSelectedTime + TimeSpan.FromSeconds(10))
                                                {
                                                    deselectLight();
                                                }
                                            }


                                            /*if (angleToLight < 0.3)
                                            {

                                                System.Diagnostics.Debug.WriteLine("POINTED!!!!!!");
                                            
                                                light1PointedAt = true;
                                            }*/

                                            firstFound = true; //don't need to react to the first user anymore
                                            
                                        }
                                    }
                                    this.DrawBody(joints, jointPoints, dc);

                                    this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                    this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);



                                    
                                }


                            }

                            //DepthSpacePoint lightPointConverted = this.coordinateMapper.MapCameraPointToDepthSpace(light1);
                            //Point light1Point = new Point(light1PointConverted.X, light1PointConverted.Y);
                            //this.DrawLight(light1Point, dc, light1PointedAt);
                            this.DrawLights(dc);
                            // prevent drawing outside of our render area
                            this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                        }
                    }
                }
            }
            catch (Exception)
            {
                System.Diagnostics.Debug.Write("exception caught");

                // ignore if the frame is no longer available
            }
            finally
            {
                // MultiSourceFrame, DepthFrame, ColorFrame, BodyIndexFrame are IDispoable
                

                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                    depthFrame = null;
                }

                if (bodyFrame != null)
                {
                    bodyFrame.Dispose();
                    bodyFrame = null;
                }

                if (multiSourceFrame != null)
                {
                    multiSourceFrame.Dispose();
                    multiSourceFrame = null;
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext)
        {
            // Draw the bones

            //System.Diagnostics.Debug.Write("DrawBody");

            // Torso
            this.DrawBone(joints, jointPoints, JointType.Head, JointType.Neck, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.Neck, JointType.SpineShoulder, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.SpineMid, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.SpineMid, JointType.SpineBase, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipLeft, drawingContext);

            // Right Arm    
            this.DrawBone(joints, jointPoints, JointType.ShoulderRight, JointType.ElbowRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.ElbowRight, JointType.WristRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.HandRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.HandRight, JointType.HandTipRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.ThumbRight, drawingContext);

            // Left Arm
            this.DrawBone(joints, jointPoints, JointType.ShoulderLeft, JointType.ElbowLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.ElbowLeft, JointType.WristLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.HandLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.HandLeft, JointType.HandTipLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.ThumbLeft, drawingContext);

            // Right Leg
            this.DrawBone(joints, jointPoints, JointType.HipRight, JointType.KneeRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.KneeRight, JointType.AnkleRight, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.AnkleRight, JointType.FootRight, drawingContext);

            // Left Leg
            this.DrawBone(joints, jointPoints, JointType.HipLeft, JointType.KneeLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.KneeLeft, JointType.AnkleLeft, drawingContext);
            this.DrawBone(joints, jointPoints, JointType.AnkleLeft, JointType.FootLeft, drawingContext);

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == TrackingState.Inferred &&
                joint1.TrackingState == TrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        private void DrawLights( DrawingContext drawingContext)
        {
            for (int i = 0; i < lightPositions.Length; i++)
            {
                DepthSpacePoint lightPointConverted = this.coordinateMapper.MapCameraPointToDepthSpace(lightPositions[i]);
                Point lightPoint = new Point(lightPointConverted.X, lightPointConverted.Y);
                //System.Diagnostics.Debug.WriteLine("light drawing:" + i + lightPoint);
                            
                if (i == curSelectedLight)
                {

                    drawingContext.DrawEllipse(this.lightPointedBrush, null, lightPoint, LightSize, LightSize);
                }
                else
                {
                    drawingContext.DrawEllipse(this.lightBrush, null, lightPoint, LightSize, LightSize);
                }
            }

        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }
    }
}
