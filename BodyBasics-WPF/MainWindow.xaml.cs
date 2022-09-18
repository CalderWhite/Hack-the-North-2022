//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Security.Cryptography.X509Certificates;
    using System.Speech.Synthesis;
    using System.Speech.Recognition;
    using System.Text.RegularExpressions;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Media3D;
    using System.Windows.Navigation;
    using System.Windows.Shapes;
    using Microsoft.Kinect;
    using Microsoft.Kinect.VisualGestureBuilder;
  
    using System.Net;
    using Newtonsoft.Json;


    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

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
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private readonly Pen specialPen = new Pen(Brushes.Orange, 10);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

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
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        public String LeftAngleText { get; set; }
        public String RightAngleText { get; set; }

        private SpeechSynthesizer synthesizer;
        private bool badLeft;
        private bool badRight;
        private bool alreadyBad = false;
        private Double firstBad = 0;
        private Double lastExport = 0;

        private bool spine_alreadyBad = false;
        private Double spine_firstBad = 0;

        private bool badSpine;
        private VisualGestureBuilderFrameSource vgbFrameSource;
        private VisualGestureBuilderFrameReader vgbFrameReader;

        private string checkType = "";
        private bool inCoolDown = false;
        private Double coolDownStart = 0;

        private int errorCount = 0;
        private string patientName = "Calder White"; // you really looked in the source code to see if this was hard coded eh? :))

        private SpeechRecognitionEngine recognizer;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            {
                // Torso
                {
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));
                }

                // Right Arm
                {
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));
                }

                // Left Arm
                {
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
                    this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));
                }


                // Right Leg
                this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

                // Left Leg
                this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft)); // we want this one
                this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft)); // want this one


                // populate body colors, one for each BodyIndex
                this.bodyColors = new List<Pen>();

                this.bodyColors.Add(new Pen(Brushes.Red, 6));
                this.bodyColors.Add(new Pen(Brushes.Orange, 6));
                this.bodyColors.Add(new Pen(Brushes.Green, 6));
                this.bodyColors.Add(new Pen(Brushes.Blue, 6));
                this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
                this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            }

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // NOTE: HTN custom code begins
            this.synthesizer = new SpeechSynthesizer();
            this.synthesizer.SetOutputToDefaultAudioDevice();
            this.synthesizer.SelectVoice("Microsoft Zira Desktop");


            this.vgbFrameSource = new VisualGestureBuilderFrameSource(this.kinectSensor, 0);
            if (this.vgbFrameSource != null)
            {
                VisualGestureBuilderDatabase database = new VisualGestureBuilderDatabase("C:\\Users\\chltn\\Documents\\Kinect Studio\\Repository\\CurlV1.gba");
                if (database != null)
                {
                    foreach (Gesture gesture in database.AvailableGestures)
                    {
                        vgbFrameSource.AddGesture(gesture);
                        Console.WriteLine($"Loading: {gesture.Name}");
                    }
                }

                this.vgbFrameReader = this.vgbFrameSource.OpenReader();
                if (this.vgbFrameReader != null)
                {
                    this.vgbFrameReader.IsPaused = false;
                    this.vgbFrameReader.FrameArrived += vgbFrameReader_FrameArrived;
                }

                Console.WriteLine($"vgbFrameReader: {this.vgbFrameReader}");

            }


            // initialize the components (controls) of the window
            this.InitializeComponent();

            this.recognizer = new SpeechRecognitionEngine(new System.Globalization.CultureInfo("en-US"));
            this.recognizer.LoadGrammar(CreateAppGrammar());
            this.recognizer.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(recognizer_SpeechRecognized);
            recognizer.SetInputToDefaultAudioDevice();

            var a = SpeechRecognitionEngine.InstalledRecognizers()[0];
            Console.WriteLine($"{a.Name} {a.Culture} {a.Id}");

            // Start asynchronous, continuous speech recognition.  
            recognizer.RecognizeAsync(RecognizeMode.Multiple);
        }

        private void resetState()
        {
            Console.WriteLine("RESETTING STATE");
            this.badLeft = false;
            this.badRight = false;
            this.alreadyBad = false;
            this.firstBad = 0;
            this.lastExport = 0;

            this.badSpine = false;
        }

        private void vgbFrameReader_FrameArrived(object sender, VisualGestureBuilderFrameArrivedEventArgs e)
        {
            return;
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (frame.DiscreteGestureResults != null)
                    {
                        //frame.DiscreteGestureResults
                        foreach (KeyValuePair<Gesture, DiscreteGestureResult> entry in frame.DiscreteGestureResults)
                        {
                            //Console.WriteLine(entry.Key.Name);
                            //Console.WriteLine(entry.Value.Confidence);
                            this.LeftAngleText = entry.Value.Confidence.ToString();
                            this.PropertyChanged(this, new PropertyChangedEventArgs("LeftAngleText"));

                        }
                        //Console.WriteLine(frame.DiscreteGestureResults);
                    }
                }
            }
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
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                // NOTE: Hack the North custom code starts HERE

                // (end Hack the North code, begin skeleton drawing code)
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    if (this.bodies.Length == 0)
                    {
                        this.resetState();
                    }
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.vgbFrameSource.TrackingId = body.TrackingId;
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }


                            if (checkType == "squat")
                            {
                                this.checkSquatKnees(joints);
                                // NOTE: This function accounts for whether it is a squat or curl. It is far more lenient with squats ( as it should be )
                                this.checkStability(joints);
                            } else if (checkType == "curl")
                            {
                                this.checkStability(joints);
                            }
                            this.speak_badForm();

                            Double currentTime = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                            Double exportFrequency = 1;
                            // TODO: Not sure if this modulo will work
                            if (currentTime - lastExport >= exportFrequency)
                            {
                                this.exportCurrentPose(joints);
                                lastExport = currentTime;
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

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
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft)); // we want this one
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft)); // want this one

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            if (this.badLeft)
            {
                if ((jointType0 == JointType.KneeLeft && jointType1 == JointType.AnkleLeft)
                    || (jointType0 == JointType.AnkleLeft && jointType1 == JointType.FootLeft))
                {
                    drawPen = this.specialPen;
                }
            }
            if (this.badRight)
            {
                if ((jointType0 == JointType.KneeRight && jointType1 == JointType.AnkleRight)
                    || (jointType0 == JointType.AnkleRight && jointType1 == JointType.FootRight))
                {
                    drawPen = this.specialPen;
                }
            }
            if (this.badSpine)
            {
                if ((jointType0 == JointType.SpineShoulder && jointType1 == JointType.SpineMid)   
                    || (jointType0 == JointType.SpineMid && jointType1 == JointType.SpineBase))
                {
                    drawPen = this.specialPen;
                }
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

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private Double getXZMagnitude(Vector3D vec)
        {
            return Math.Sqrt(vec.X * vec.X + vec.Z * vec.Z);
        }

        private bool checkStability(IReadOnlyDictionary<JointType, Joint> joints)
        {
            Joint hipLeftJoint = joints[JointType.HipLeft];
            Joint hipRightJoint = joints[JointType.HipRight];
            Joint spineBaseJoint = joints[JointType.SpineBase];
            Joint spineMidJoint = joints[JointType.SpineMid];
            Joint spineShoulderJoint = joints[JointType.SpineShoulder];

            Vector3D spineBase = new Vector3D(spineBaseJoint.Position.X, spineBaseJoint.Position.Y, spineBaseJoint.Position.Z);
            Vector3D spineMid = new Vector3D(spineMidJoint.Position.X, spineMidJoint.Position.Y, spineMidJoint.Position.Z);
            Vector3D spineShoulder = new Vector3D(spineShoulderJoint.Position.X, spineShoulderJoint.Position.Y, spineShoulderJoint.Position.Z);

            //this.LeftAngleText = spineBase.X.ToString();
            //this.RightAngleText = spineShoulder.X.ToString();

            //this.PropertyChanged(this, new PropertyChangedEventArgs("LeftAngleText"));
            //this.PropertyChanged(this, new PropertyChangedEventArgs("RightAngleText"));

            double spineErrorBound = 1.0;
            if (this.checkType == "squat")
            {
                spineErrorBound = 0.2;
            } else if (this.checkType == "curl")
            {
                spineErrorBound = 0.02;
            }
            this.badSpine = Math.Abs(Math.Round(spineBase.X, 2) - Math.Round(spineShoulder.X, 2)) > spineErrorBound;
            //this.badSpine = spineBase.Y != spineShoulder.Y;   

            return false;
        }

        private void speak_badForm()
        {
            Double coolDownThreshold = 5.0;
            if (this.inCoolDown)
            {
                Double currentTime = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                if (currentTime - this.coolDownStart > coolDownThreshold)
                {
                    this.inCoolDown = false;
                }

                return;
            }

            Double badSquatFormDuration = 0.6;
            if (this.alreadyBad)
            {
                if (!this.badLeft && !this.badRight)
                {
                    this.alreadyBad = false;
                }
                else
                {
                    Double currentTime = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                    if (currentTime - this.firstBad > badSquatFormDuration)
                    {
                        this.synthesizer.SpeakAsync("Try to keep your knees behind your toes!");
                        this.firstBad = currentTime;
                        this.inCoolDown = true;
                        this.coolDownStart = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                        ++this.errorCount;
                    }
                }
            }
            else
            {
                if (this.badLeft || this.badRight)
                {
                    this.alreadyBad = true;
                    this.firstBad = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                }
            }

            Double badSpineFormDuration = 0.05;
            if (this.spine_alreadyBad)
            {
                if (!this.badSpine)
                {
                    this.spine_alreadyBad = false;
                }
                else
                {
                    Double currentTime = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                    if (currentTime - this.spine_firstBad > badSpineFormDuration)
                    {
                        this.synthesizer.SpeakAsync("Try to keep your back straight!");
                        this.spine_firstBad = currentTime;
                        this.inCoolDown = true;
                        this.coolDownStart = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                        ++this.errorCount;
                    }
                }
            }
            else
            {
                if (this.badSpine)
                {
                    this.spine_alreadyBad = true;
                    this.spine_firstBad = (Double)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds / 1000;
                }
            }
        }

        private bool checkSquatKnees(IReadOnlyDictionary<JointType, Joint> joints)
        {
            Joint ankleLeftJoint = joints[JointType.AnkleLeft];
            Joint footLeftJoint = joints[JointType.FootLeft];
            Joint kneeLeftJoint = joints[JointType.KneeLeft];

            Joint ankleRightJoint = joints[JointType.AnkleRight];
            Joint footRightJoint = joints[JointType.FootRight];
            Joint kneeRightJoint = joints[JointType.KneeRight];

            Vector3D ankleLeft = new Vector3D(ankleLeftJoint.Position.X, ankleLeftJoint.Position.Y, ankleLeftJoint.Position.Z);
            Vector3D footLeft = new Vector3D(footLeftJoint.Position.X, footLeftJoint.Position.Y, footLeftJoint.Position.Z);
            Vector3D kneeLeft = new Vector3D(kneeLeftJoint.Position.X, kneeLeftJoint.Position.Y, kneeLeftJoint.Position.Z);

            Vector3D ankleRight = new Vector3D(ankleRightJoint.Position.X, ankleRightJoint.Position.Y, ankleRightJoint.Position.Z);
            Vector3D footRight = new Vector3D(footRightJoint.Position.X, footRightJoint.Position.Y, footRightJoint.Position.Z);
            Vector3D kneeRight = new Vector3D(kneeRightJoint.Position.X, kneeRightJoint.Position.Y, kneeRightJoint.Position.Z);

            double magnitudeThreshold = 0.5;
            Vector3D footVecLeft = footLeft - ankleLeft;
            Vector3D shinVecLeft = kneeLeft - ankleLeft;
            // knee is over the foot. BAD!
            this.badLeft = getXZMagnitude(shinVecLeft) > getXZMagnitude(footVecLeft) * (1 + magnitudeThreshold);
            //Double leftAngle = Vector3D.AngleBetween(footVecLeft, shinVecLeft);

            Vector3D footVecRight = footRight - ankleRight;
            Vector3D shinVecRight = kneeRight - ankleRight;
            this.badRight = getXZMagnitude(shinVecRight) > getXZMagnitude(footVecRight) * (1 + magnitudeThreshold);
            //Double rightAngle = Vector3D.AngleBetween(footVecRight, shinVecRight);

            //this.LeftAngleText = $"X:{Math.Round(ankleLeft.X, 2)}\tY:{Math.Round(ankleLeft.Y, 2)}\tZ:{Math.Round(ankleLeft.Z, 2)}";
            //this.PropertyChanged(this, new PropertyChangedEventArgs("LeftAngleText"));

            /*
            this.LeftAngleText = Math.Round(leftAngle).ToString();
            this.RightAngleText = Math.Round(rightAngle).ToString();
            if (this.PropertyChanged != null)
            {
                this.PropertyChanged(this, new PropertyChangedEventArgs("LeftAngleText"));
                this.PropertyChanged(this, new PropertyChangedEventArgs("RightAngleText"));
            }
            */

            //this.synthesizer.Speak("I love your squat form!");
            //TrackingState trackingState = joints[jointType].TrackingState;

            //if (trackingState == TrackingState.Tracked)
            //else if (trackingState == TrackingState.Inferred)
            return false;
        }

        private void exportCurrentPose(IReadOnlyDictionary<JointType, Joint> joints)
        {
            return;
            var httpWebRequest = (HttpWebRequest)WebRequest.Create("http://127.0.0.1:5000/accept_pose");
            httpWebRequest.ContentType = "application/json";
            httpWebRequest.Method = "POST";
            using (var streamWriter = new StreamWriter(httpWebRequest.GetRequestStream()))
            {

                string json = JsonConvert.SerializeObject(joints);
                streamWriter.Write(json);
            }

            var httpResponse = (HttpWebResponse)httpWebRequest.GetResponse();
            using (var streamReader = new StreamReader(httpResponse.GetResponseStream()))
            {
                var result = streamReader.ReadToEnd();
            }
        }

        public void SquatClick(object sender, EventArgs e)
        {
            this.checkType = "squat";
        }

        public void CurlClick(object sender, EventArgs e)
        {
            this.checkType = "curl";
        }

        public void StartWorkoutClick(object sender, EventArgs e)
        {
            this.errorCount = 0;
        }

        public void StopWorkoutClick(object sender, EventArgs e)
        {
            StopWorkout();
        }

        private void StopWorkout()
        {
            var httpWebRequest = (HttpWebRequest)WebRequest.Create("http://127.0.0.1:5000/accept_workout");
            httpWebRequest.ContentType = "application/json";
            httpWebRequest.Method = "POST";
            using (var streamWriter = new StreamWriter(httpWebRequest.GetRequestStream()))
            {
                Dictionary<string, dynamic> obj = new Dictionary<string, dynamic>();
                obj["errorCount"] = this.errorCount;
                obj["patientName"] = this.patientName;
                string json = JsonConvert.SerializeObject(obj);
                streamWriter.Write(json);
            }

            var httpResponse = (HttpWebResponse)httpWebRequest.GetResponse();
            using (var streamReader = new StreamReader(httpResponse.GetResponseStream()))
            {
                var result = streamReader.ReadToEnd();
                Console.WriteLine(result);
            }
            this.checkType = "";
            resetState();
        }

        private void recognizer_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            Console.WriteLine("Recognized text: " + e.Result.Text);
            string t = e.Result.Text;
            if (t == "Start squat")
            {
                this.checkType = "squat";
            } else if (t == "Start curl")
            {
                this.checkType = "curl";
            }
            else if (t == "Start workout")
            {
                this.errorCount = 0;
            }
            else if (t == "Stop workout")
            {
                StopWorkout();
            }
            else if (t == "Pause Workout")
            {
                this.checkType = "";
                resetState();
            }
        }

        private Grammar CreateAppGrammar()
        {

            //Choices colorChoice = new Choices(new string[] { "red", "green", "blue" });
            //GrammarBuilder colorElement = new GrammarBuilder(colorChoice);

            // Create grammar builders for the two versions of the phrase.  
            GrammarBuilder squat = new GrammarBuilder("Start squat");
            GrammarBuilder curl = new GrammarBuilder("Start curl");
            GrammarBuilder startWorkout = new GrammarBuilder("Start workout");
            GrammarBuilder stopWorkout = new GrammarBuilder("Stop workout");
            GrammarBuilder pauseWorkout = new GrammarBuilder("Pause Workout");

            // Create a Choices for the two alternative phrases, convert the Choices  
            // to a GrammarBuilder, and construct the grammar from the result.  
            Choices bothChoices = new Choices(new GrammarBuilder[] { squat, curl, startWorkout, stopWorkout, pauseWorkout });
            Grammar grammar = new Grammar((GrammarBuilder)bothChoices);
            grammar.Name = "HackTheNorth2022";
            return grammar;
        }
        private Grammar CreateColorGrammar()
        {

            // Create a set of color choices.  
            Choices colorChoice = new Choices(new string[] { "red", "green", "blue" });

            // Create grammar builders for the two versions of the phrase.  
            GrammarBuilder makePhrase =
              GrammarBuilder.Add((GrammarBuilder)"Make background", colorChoice);
            GrammarBuilder setPhrase =
              GrammarBuilder.Add("Set background to", (GrammarBuilder)colorChoice);

            // Create a Choices for the two alternative phrases, convert the Choices  
            // to a GrammarBuilder, and construct the grammar from the result.  
            Choices bothChoices = new Choices(new GrammarBuilder[] { makePhrase, setPhrase });
            GrammarBuilder bothPhrases = new GrammarBuilder(bothChoices);

            Grammar grammar = new Grammar(bothPhrases);
            grammar.Name = "backgroundColor";
            return grammar;
        }
    }
}
