//using ABI.System;
using Accord.Math.Geometry;
using Accord.Math.Kinematics;
using MathNet.Numerics;
using MathNet.Numerics.IntegralTransforms;
using MathNet.Numerics.LinearRegression;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Configuration;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Media;
using System.Reflection;
using System.Runtime.InteropServices;
//using System.Numerics;
using System.Security.Policy;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media.Animation;
//using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using VisualComponents.Create3D;
using Windows.System;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;
using System.Net.Sockets;
using System.Net;
using CleanScanOptimizePath_Net4;

namespace RobotScannerCombineNet6
{
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        Random RND = new Random();

        #region Default values - readed from config

        public double StartSearchIncr;
        public double IncrementLimit;
        public double IncDescentDiv;

        string URI_SphereFileName;
        string URI_InputFiles;
        string URI_OutputFile;

        double[] TableDH = { 350, 0, 0, 0, 90, -815, 850, 0, 0, 145, 90, 0, 0, -90, -820, 0, 90, 0, 170, 80, 0, 125, 0, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


        double[] ExpectedSpherePos = null;
        double ExpectedSphereRadius = 0;
        bool GetExpectedRadiusFromLines = false;
        double SurfaceTolerance = 0.1;
        double ScanlineMinMaxClear = 0.01;
        double StopError = 0.20;
        List<double[]> ExpectedSpheresPositions;

        #endregion Default values - readed from config

        #region Main Variables
        bool ImportTCP0 = false;
        bool IsEffective = true;
        bool InProgress = false;
        bool Intro = false;
        bool Sounds = false;
        Matrix Scanner_MTX;

        int SmoothPointCountDiv = 20;
        double SL_OrderDivisor = 10;
        double XReshakeOffset = -7;
        int[] DefaultSkipIndexes = { 0, 2, 5 };
        double DegToRad = Math.PI / 180;

        int graphOffset = 0;
        List<Task> ActiveProcesses = null;
        bool PointCloud_Updated = false;
        bool isRenderReady = false;
        #endregion Main Variables


        static int BufferSize = 40970;
        byte[] BufferXZPC = new byte[BufferSize];
        byte[] BufferRPC = new byte[BufferSize];


        static IPAddress IP_Addrs = IPAddress.Parse("192.168.0.20");
        IPEndPoint IP_EndPointXZPC = new IPEndPoint(IP_Addrs, 6000);
        IPEndPoint IP_EndPointRPC = new IPEndPoint(IP_Addrs, 6001);


        NetworkStream StreamXZPC = null;
        NetworkStream StreamRPC = null;

        public MainWindow()
        {
            //TestVoid();
            //return;

            ReadConfigValues();
            InitializeComponent();


            WindowStartupLocation = WindowStartupLocation.CenterScreen;

            if (Intro)
            {
                AllowsTransparency = true;
                WindowStyle = WindowStyle.None;
                Opacity = 0;

                DoubleAnimation windowAnimation = new DoubleAnimation(0, 1, TimeSpan.FromSeconds(4));
                this.BeginAnimation(OpacityProperty, windowAnimation);

                if (Sounds)
                {
                    SoundPlayer soundPlayer = new SoundPlayer(Properties.Resources.FadeInSound);
                    soundPlayer.Play();
                }
            }
            DataContext = this;
            OutFileCounter = 0;
            Buttons_Enabled = true;
            STOPBUTTON.IsEnabled = true;

            Model = new Model3DGroup();
            ViewPort.LayoutUpdated += ViewPort_LayoutUpdated;
        }

        private void ViewPort_LayoutUpdated(object? sender, EventArgs e)
        {
            isRenderReady = true;
            //if (PointCloud_Updated == true)
            //{
            //    //Visual3DCollection visual3Ds = ViewPort.Children;
            //    //visual3Ds.Add(PointCloudVisualModel);
            //    Rect3D pc_Bounds = Visual3DHelper.FindBounds(ViewPort.Children);
            //
            //    if (pc_Bounds != Rect3D.Empty)
            //{
            //    ViewPort.BringIntoView(new Rect(pc_Bounds.X, pc_Bounds.Y, 1, 1));
            //}
            //    PointCloud_Updated = false;
            //}
        }

        #region Main functions

        bool ReadConfigValues()
        {
            bool status = true;
            try { StartSearchIncr = double.Parse(ConfigurationManager.AppSettings["StartSearchIncr"]); }
            catch { status = false; }
            try { IncrementLimit = double.Parse(ConfigurationManager.AppSettings["IncrementLimit"]); }
            catch { status = false; }
            try { IncDescentDiv = double.Parse(ConfigurationManager.AppSettings["IncDescentDiv"]); }
            catch { status = false; }
            try { IncDescentDiv = double.Parse(ConfigurationManager.AppSettings["IncDescentDiv"]); }
            catch { status = false; }
            try { URI_SphereFileName = ConfigurationManager.AppSettings["URI_SphereFileName"]; }
            catch { status = false; }
            try { URI_InputFiles = ConfigurationManager.AppSettings["URI_InputFiles"]; }
            catch { status = false; }
            try { URI_OutputFile = ConfigurationManager.AppSettings["URI_OutputFile"]; }
            catch { status = false; }
            try { SurfaceTolerance = double.Parse(ConfigurationManager.AppSettings["SurfaceTolerance"]); }
            catch { status = false; }
            try { ScanlineMinMaxClear = double.Parse(ConfigurationManager.AppSettings["ScanlineMinMaxClear"]); }
            catch { status = false; }
            try { StopError = double.Parse(ConfigurationManager.AppSettings["StopError"]); }
            catch { status = false; }
            try { XReshakeOffset = double.Parse(ConfigurationManager.AppSettings["XReshakeOffset"]); }
            catch { status = false; }
            try { ExpectedSphereRadius = double.Parse(ConfigurationManager.AppSettings["ExpectedSphereRadius"]); }
            catch { status = false; }
            try { ImportTCP0 = bool.Parse(ConfigurationManager.AppSettings["ImportTCP0"]); }
            catch { status = false; }
            try { Intro = bool.Parse(ConfigurationManager.AppSettings["Intro"]); }
            catch { status = false; }
            try { Sounds = bool.Parse(ConfigurationManager.AppSettings["Sounds"]); }
            catch { status = false; }



            if (ExpectedSphereRadius == 0)
            {
                GetExpectedRadiusFromLines = true;
            }


            {
                string strDH = ConfigurationManager.AppSettings["TableDH"];
                string[] arrDH = strDH.Split(',');
                TableDH = new double[arrDH.Length];

                try
                {
                    double[] tempTableDH = new double[TableDH.Length];
                    for (int i = 0; i < arrDH.Length; i++)
                    {
                        tempTableDH[i] = double.Parse(arrDH[i]);
                    }
                    tempTableDH.CopyTo(TableDH, 0);
                }
                catch
                {
                    status = false;
                }
            } // Table DH
            
            
            {
                string expectedSpheresPositionsSTR = ConfigurationManager.AppSettings["ExpectedSpheresPositions"];
                string[] expectedSpheresPositionsARR = expectedSpheresPositionsSTR.Split(';');
                ExpectedSpheresPositions = new List<double[]>();

                try
                {
                    for (int i = 0; i < expectedSpheresPositionsARR.Length; i++)
                    {
                        string[] spherePosARR = expectedSpheresPositionsARR[i].Split(",");
                        if (spherePosARR.Length != 3) { continue; }

                        double[] newPos = new double[spherePosARR.Length];
                        for (int j = 0; j < spherePosARR.Length; j++)
                        {
                            newPos[j] = double.Parse(spherePosARR[j]);
                        }
                        ExpectedSpheresPositions.Add(newPos);
                    }
                }
                catch
                {
                    ExpectedSpheresPositions = null;
                    status = false;
                }


                //try
                //{
                //    double[] tempTableDH = new double[TableDH.Length];
                //    for (int i = 0; i < arrDH.Length; i++)
                //    {
                //        tempTableDH[i] = double.Parse(arrDH[i]);
                //    }
                //    tempTableDH.CopyTo(TableDH, 0);
                //}
                //catch
                //{
                //    status = false;
                //}
            } // ExpectedSpheresPositions


            return status;
        }

        void plcData_2_PointCloud_TCPIP()
        {
            TcpClient client = new();
            client.Connect(IP_EndPointXZPC);
            StreamXZPC = client.GetStream();

        }

        void plcData_2_PointCloud_File()
        {
            ProgresActionName = "Started";

            if (!Directory.Exists(URI_InputFiles))
            {
                ProgresActionName = string.Format("Directory {0} does not exist!", URI_InputFiles);
                Buttons_Enabled = true;
                InProgress = false;
                return;
            }
            string[] allFilesDH = Directory.GetFiles(URI_InputFiles, "*.dh");
            if (allFilesDH.Length == 0)
            {
                ProgresActionName = string.Format("Directory {0} does not contains *.dh file", URI_InputFiles);
                Buttons_Enabled = true;
                InProgress = false;
                return;
            }
            DHZones dhZones = new DHZones(allFilesDH[0]);

            string[] allFilesURI = Directory.GetFiles(URI_InputFiles, "*.rpc");
            Array.Reverse(allFilesURI);
            //URI_InputFiles += "2023-04-05-17-42-14";

            ActiveProcesses = new List<Task>();

            foreach (string uri in allFilesURI)
            {
                var newUri = uri.Substring(0, uri.Length - 4);
                (int[] robotTemperatures, double[][] robotPoses, Matrix[][] scanLines) = readPLCData(newUri);

                if (robotPoses == null) { continue; }


                OutFileCounter++;
                string newURI = URI_OutputFile + "_" + OutFileCounter + ".txt";
                while (File.Exists(newURI))
                {
                    OutFileCounter++;
                    newURI = URI_OutputFile + "_" + OutFileCounter + ".txt";
                }
                write3DPointCloudTXT(newURI, robotTemperatures, robotPoses, scanLines, 0.2, dhZones);
                //WritePLYbyTriggers(newURI, robotPoses, scanLines, TableDH);

                //break;
            }

            Buttons_Enabled = true;
            InProgress = false;
        }

        void calibrationOnSphere(bool exportReport = true, bool exportOrig = true, bool exportClear = true, bool writeDH = false, bool lastOperation = true)
        {
            ImportTCP0 = false;

            string uri_InputFile = URI_InputFiles + URI_SphereFileName;
            (int[] robotTemperatures, double[][] robotPoses, Matrix[][] scan_Lines) = readPLCData(uri_InputFile);

            if (robotPoses == null)
            {
                Buttons_Enabled = true;
                InProgress = false;
                return;
            }

            //robotPoses = jointAntiBackLash(robotPoses); ////////////// <<<<<<<<<<<<<<<<<<<<<===================


            (robotPoses, scan_Lines) = cleanInputData(robotPoses, scan_Lines, TableDH, 20, 1, 0.2);
            Matrix[] pointCloud = generate3DPointCloud(robotPoses, scan_Lines, TableDH);
            //var sphere = evaluateSingleSphereData(pointCloud, ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance);
            //WriteOutputPLY(URI_OutputFile + "_calibData.ply", pointCloud);

            var calibResult = singleSphereCalibration(robotPoses, scan_Lines);
            TableDH = calibResult.resultDH;

            if (exportReport)
            {
                pointCloud = generate3DPointCloud(robotPoses, scan_Lines, TableDH);
                writeCalibReport(URI_OutputFile + ".report", robotPoses, scan_Lines, uri_InputFile, ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance, calibResult.resultDH, true);
            }
            if (exportClear)
            {
                WriteOutputPLY(URI_OutputFile + "_C.txt", pointCloud);

                //WritePLYbyTriggers(URI_OutputFile + "C_Trig.txt", robotPoses, scan_Lines, TableDH);
            }

            
            (robotTemperatures, robotPoses, scan_Lines) = readPLCData(uri_InputFile);
            if (exportOrig)
            {
                //for (int i = 0; i < scan_Lines.Length; i++)
                //{
                //    scan_Lines[i] = clearScanlinesInPolynom(scan_Lines[i], ScanlineMinMaxClear);
                //}

                WriteOutputPLY(URI_OutputFile + "_O.txt", generate3DPointCloud(robotPoses, scan_Lines, TableDH));

                //WritePLYbyTriggers(URI_OutputFile + "O_Trig.txt", robotPoses, scan_Lines, TableDH);
            }
            if (lastOperation)
            {
                Buttons_Enabled = true;
                InProgress = false;

                ProgresActionName = "Single sphere calibration - Done";

                if (Sounds)
                {
                    SoundPlayer soundPlayer = new SoundPlayer(Properties.Resources.FinishedSound);
                    soundPlayer.Play();
                }
            }
        }

        void calibrationOnMultiSphere()
        {
            #region Load and prepare data

            if (ExpectedSpheresPositions.Count == 0)
            {
                ProgresActionName = "ExpectedSpheresPositions is required for calibration";
                Buttons_Enabled = true;
                InProgress = false;
                return;
            }
            ImportTCP0 = false;

            string[] allFilesURI = Directory.GetFiles(URI_InputFiles, "*.rpc");
            //Array.Reverse(allFilesURI);

            if (allFilesURI.Length < 0)
            {
                Buttons_Enabled = true;
                InProgress = false;
                return;
            }

            List<(double[][] robotPoses, Matrix[][] scan_Lines)> spheres = new List<(double[][] robotPoses, Matrix[][] scan_Lines)>();
            foreach (string uri in allFilesURI)
            {
                var newUri = uri.Substring(0, uri.Length - 4);
                var newData = readPLCData(newUri);

                if (newData.robotPoses != null)
                {
                    spheres.Add((newData.robotPoses, newData.scanLines));
                }
            }

            if (ExpectedSpheresPositions.Count != spheres.Count)
            {
                ProgresActionName = "Count of ExpectedSpheresPositions is not the same as count of measured spheres";
                Buttons_Enabled = true;
                InProgress = false;
                return;
            }

            List<(double[][] robotPoses, Matrix[][] scan_Lines)> clearSpheres = new List<(double[][] robotPoses, Matrix[][] scan_Lines)>();
            for (int i = 0; i < spheres.Count; i++)
            {
                var clearData = cleanInputData(spheres[i].robotPoses, spheres[i].scan_Lines, TableDH, 10, 1, 0.2);
                clearSpheres.Add((clearData.robotPoses, clearData.scan_Lines));

                WriteOutputPLY(URI_OutputFile + string.Format("_clean_{0}.ply", i), generate3DPointCloud(clearData.robotPoses, clearData.scan_Lines, TableDH));
            }

            #endregion


            #region Common calibration - Without Positions
            ProgresActionName = "Searching common params: Without positions, DH + TCP";
            int numSphere = 0;
            List<CalibSphere> toEvalSpheres = new List<CalibSphere>();
            foreach (var sphere in clearSpheres)
            {
                toEvalSpheres.Add(new CalibSphere()
                {
                    robotPoses = sphere.robotPoses,
                    scan_Lines = sphere.scan_Lines,
                    expectedPosition = null,
                    expectedRadius = ExpectedSphereRadius,
                    surfaceTolerance = SurfaceTolerance
                });
                numSphere++;
            }

            StartSearchIncr = 5;
            IncrementLimit = 0.0001;
            DefaultSkipIndexes = new int[] { 0, 2, 5, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};
            var calibResult = multiSphereCalibration(toEvalSpheres);
            TableDH = calibResult.resultDH;
            
            
            StartSearchIncr = 5;
            IncrementLimit = 0.00000000000001;
            ProgresActionName = "Searching common params: Without positions, only direction offsets"; 
            DefaultSkipIndexes = Enumerable.Range(0, 34).Select(i => (int)i).ToArray();
            calibResult = multiSphereCalibration(toEvalSpheres);
            TableDH = calibResult.resultDH;
            #endregion

            #region Common calibration - With Positions
            ProgresActionName = "Searching common params: With positions, Offsets + Base + ScanShifts";
            numSphere = 0;
            List<CalibSphere> toEvalSpheresPoses = new List<CalibSphere>();
            foreach (var sphere in clearSpheres)
            {
                toEvalSpheresPoses.Add(new CalibSphere()
                {
                    robotPoses = sphere.robotPoses,
                    scan_Lines = sphere.scan_Lines,
                    expectedPosition = ExpectedSpheresPositions[numSphere],
                    expectedRadius = ExpectedSphereRadius,
                    surfaceTolerance = SurfaceTolerance
                });
                numSphere++;
            }

            StartSearchIncr = 25;
            IncrementLimit = 0.0001;
            DefaultSkipIndexes = Enumerable.Range(0, 25).Select(i => (int)i).ToArray();
            calibResult = multiSphereCalibration(toEvalSpheresPoses);
            TableDH = calibResult.resultDH;
            #endregion

            #region Report and Export common results
            int num = 0;
            foreach (var sphere in spheres)
            {
                var pointCloud = generate3DPointCloud(sphere.robotPoses, sphere.scan_Lines, TableDH);
                writeCalibReport(URI_OutputFile + "_Multi_" + num + ".report", sphere.robotPoses, sphere.scan_Lines, allFilesURI[num], toEvalSpheresPoses[num].expectedPosition, toEvalSpheresPoses[num].expectedRadius, toEvalSpheresPoses[num].surfaceTolerance, TableDH);
                WriteOutputPLY(URI_OutputFile + "_Multi_Calib_" + num + ".txt", pointCloud);
                num++;
            }
            #endregion

            #region Separate sphere calibration

            string OrigOutName = URI_OutputFile;
            double[] commonDH = new double[TableDH.Length];
            TableDH.CopyTo(commonDH, 0);
            num = 0;

            //foreach (string uri in allFilesURI)
            for (int i = 0; i <allFilesURI.Length; i += 1)
            {
                #region 1. config

                string[] splitted = allFilesURI[i].Split('\\');
                int lastIndex = splitted.Length - 1;
                URI_SphereFileName = splitted[lastIndex].Substring(0, splitted[lastIndex].Length - 4);
                URI_OutputFile = OrigOutName + "_" + num;

                ProgresActionName = string.Format("Searching sphere {0} parameters: DH + Axis offsets", num);
                ExpectedSpherePos = null;
                StartSearchIncr = 5;
                IncrementLimit = 0.0001;
                DefaultSkipIndexes = new int[] { 0, 2, 5, 19, 20, 21, 22, 23, 24, 28, 29, 30, 31, 32, 33, 34, 35, 36};

                calibrationOnSphere(false, false, false, false, false);


                ProgresActionName = string.Format("Searching sphere {0} parameters: only direction scanShifts", num);
                ExpectedSpherePos = null;
                StartSearchIncr = 5;
                IncrementLimit = 0.00000000000001;
                DefaultSkipIndexes = Enumerable.Range(0, 34).Select(i => (int)i).ToArray();

                calibrationOnSphere(false, false, false, false, false);

                ProgresActionName = string.Format("Searching sphere {0} parameters: Offsets + Base + ScanShifts", num);
                ExpectedSpherePos = toEvalSpheresPoses[num].expectedPosition;
                StartSearchIncr = 5;
                IncrementLimit = 0.0001;
                DefaultSkipIndexes = Enumerable.Range(0, 25).Select(i => (int)i).ToArray();

                calibrationOnSphere(false, false, false, false, false);

                ProgresActionName = string.Format("Searching sphere {0} parameters: full range", num);
                ExpectedSpherePos = toEvalSpheresPoses[num].expectedPosition;
                StartSearchIncr = 5;
                IncrementLimit = 0.0001;
                DefaultSkipIndexes = new int[] { 0, 2, 5 };
                
                calibrationOnSphere(true, true, true, true, false);

                #endregion 1. config

                if (false)
                {
                    #region 2. config

                    ProgresActionName = string.Format("Searching sphere {0} parameters: DH + Axis offsets - Precise Mode", num);
                    StartSearchIncr = 1;
                    IncrementLimit = 0.00001;
                    DefaultSkipIndexes = new int[] { 0, 2, 5, 19, 20, 21, 22, 23, 24, 28, 29, 30, 31, 32, 33, 34, 35, 36 };

                    calibrationOnSphere(false, false, false, false, false);

                    ProgresActionName = string.Format("Searching sphere {0} parameters: only direction scanShifts - Precise Mode", num);
                    StartSearchIncr = 1;
                    IncrementLimit = 0.00000000000001;
                    DefaultSkipIndexes = Enumerable.Range(0, 34).Select(i => (int)i).ToArray();

                    calibrationOnSphere(false, false, false, false, false);

                    ProgresActionName = string.Format("Searching sphere {0} parameters: Offsets + Base + ScanShifts", num);
                    StartSearchIncr = 1;
                    IncrementLimit = 0.0001;
                    DefaultSkipIndexes = Enumerable.Range(0, 25).Select(i => (int)i).ToArray();

                    calibrationOnSphere(false, false, false, false, false);

                    ProgresActionName = string.Format("Searching sphere {0} parameters: full range - Precise Mode", num);
                    StartSearchIncr = 1;
                    IncrementLimit = 0.0000001;
                    DefaultSkipIndexes = new int[] { 0, 2, 5 };

                    calibrationOnSphere(true, true, true, true, false);

                    #endregion 2. config
                }
                
                commonDH.CopyTo(TableDH, 0);
                num++;
            }

            #endregion

            #region Finish - Useless things like: sounds, ...
            if (Sounds)
            {
                SoundPlayer soundPlayer = new SoundPlayer(Properties.Resources.FinishedSound);
                soundPlayer.Play();
            }

            Buttons_Enabled = true;
            InProgress = false;

            ProgresActionName = "Multi sphere calibration - Done";
            #endregion

            // Hybernate PC !!!!
            //SetSuspendState(true, true, true);
        }

        int[] rangeIndexes(int start, int end, int[] exceptThis = null)
        {
            List<int> returnIndexes = new List<int>();
            for (int i = start; i < end; i++)
            {
                if (exceptThis.Contains(i))
                {
                    continue;
                }
                returnIndexes.Add(i);
            }

            return returnIndexes.ToArray();
        }

        #endregion

        #region Data preparation for calibration

        void FFT_Testy()
        {
            string[] allFilesURI = Directory.GetFiles(URI_InputFiles, "*.rpc");

            var newUri = allFilesURI[0].Substring(0, allFilesURI[0].Length - 4);
            (int[] robotTemperatures, double[][] robotPoses, Matrix[][] scanLines) = readPLCData(newUri);

            List<double> middlePoins = new List<double>();
            for (int i = 0; i < scanLines.Length; i++)
            {
                if (scanLines[i].Length < 10)
                {
                    continue;
                }
                int width = scanLines[i].Length;
                middlePoins.Add(scanLines[i][width / 2].Pz);

                //if (middlePoins.Count == 250) { break; }
            }

            int numSamples = middlePoins.Count; // (int)(DHPolyLine.Width);
            int sampleRate = 4096;
            double[] sinus_1 = middlePoins.ToArray();// Generate.Sinusoidal(numSamples, sampleRate, 10, 20);
            double[] sinus_2 = Generate.Sinusoidal(numSamples, sampleRate, 1024, 1, 0, 0.1);
            double[] resultSinus = new double[numSamples];

            DHPoints = new System.Windows.Media.PointCollection();
            System.Windows.Media.PointCollection points = new System.Windows.Media.PointCollection();

            System.Numerics.Complex[] samples = new System.Numerics.Complex[numSamples];
            for (int i = 0; i < numSamples; i++)
            {
                samples[i] = new System.Numerics.Complex(sinus_1[i] + sinus_2[i], 0);
                resultSinus[i] = samples[i].Real;

                //points.Add(new Point(i, samples[i].Real));
            }

            Fourier.Forward(samples, FourierOptions.NoScaling);
                        
            points.Add(new Point(numSamples, 0));
            points.Add(new Point(0, 0));

            double[] imags = new double[numSamples];
            double[] mags = new double[numSamples];
            int maxFreqIndex = numSamples / 2;

            //int head = numSamples / 3;
            //int tale = numSamples - head;

            double HzPerRate = sampleRate / numSamples;
            for (int i = 0; i < numSamples / 2; i++)
            {
                double mag = samples[i].Magnitude;
                imags[i] = samples[i].Imaginary;
                

                //if (i > head && i < tale )//&& mag > 50)
                //{
                //    samples[i] = new System.Numerics.Complex(1, 0 );
                //    mag = samples[i].Magnitude;
                //}
                mags[i] = mag;
            }
            //var min = mags.Min();
            //var max = mags.Max();

            //for (int i = 0; i < numSamples; i++)
            //{
            //    points.Add(new Point(i, remapValue(mags[i / 10], min, max, DHPolyLine.Height-10, 10)));
            //}

            //samples[16] = new System.Numerics.Complex(600, 0);
            //samples[17] = new System.Numerics.Complex(600, 0);
            //samples[18] = new System.Numerics.Complex(600, 0);
            //samples[19] = new System.Numerics.Complex(600, 0);

            double[] newZvals = new double[numSamples];
            Fourier.Inverse(samples, FourierOptions.AsymmetricScaling);
            for (int i = 0; i < numSamples; i++)
            {
                newZvals[i] = samples[i].Real;
                //points.Add(new Point(i, 150 + samples[i].Real));
            }

            DHPoints = points;
        }

        (double[][] robotPoses, Matrix[][] scan_Lines) cleanInputData (double[][] robotPoses, Matrix[][] scan_Lines, double[] tableDH, int maxLinePointCount, int motionFilterOrder,  double cutOutRatio)
        {
            if (true)
            {
                List<Matrix> optimizedTrajectory = new List<Matrix>();
                List<Matrix> originalTrajectory = new List<Matrix>();
                List<double> timeStamps = new List<double>();
                Matrix[] optimizedSubPath = null;

                int j = 0;
                for (int i = 0; i < robotPoses.Length; i++)
                {
                    originalTrajectory.Add(extendedDHCalculations(tableDH, robotPoses[i]));
                    timeStamps.Add(robotPoses[i][0]);
                    if (i < robotPoses.Length - 1)
                    {
                        //Matrix nextPoint_old = MyDHCalculations_old(new double[] { 350, 180, 0, 90, -815, 850, 0, 145, 90, -90, -90, -820, 0, 90, 180, 170, 0, 0, 0, 0, 80, 0, 125, 0, 90, 0 }, robotPoses[i + 1]);
                        Matrix nextPoint = extendedDHCalculations(tableDH, robotPoses[i + 1]);
                        var dist = distance(originalTrajectory[j].Px, originalTrajectory[j].Py, originalTrajectory[j].Pz, nextPoint.Px, nextPoint.Py, nextPoint.Pz);

                        if (dist > 10)
                        {
                            optimizedSubPath = filterMotionCloud(originalTrajectory.ToArray(), motionFilterOrder, cutOutRatio, 50); // 10 polôh začiatku a konca odseknúť - vlny
                            optimizedTrajectory.AddRange(optimizedSubPath);

                            originalTrajectory = new List<Matrix>();
                            timeStamps = new List<double>();
                            j = -1;
                        }
                    }
                    j++;

                    if (i > 0) { ProgressValue = (i / (float)robotPoses.Length) * 100; }
                }

                optimizedSubPath = filterMotionCloud(originalTrajectory.ToArray(), motionFilterOrder, cutOutRatio, 20); // 10 polôh začiatku a konca odseknúť - vlny
                optimizedTrajectory.AddRange(optimizedSubPath);


                // Remove robotPoses and belonging scanLines
                List<double[]> newRobotPoses = new List<double[]>();
                List<Matrix[]> newScanLines = new List<Matrix[]>();

                for (int i = 0; i < scan_Lines.Length; i += 4)
                {
                    if (optimizedTrajectory[i].Px != 0)
                    {
                        newRobotPoses.Add(robotPoses[i]);
                        newScanLines.Add(scan_Lines[i]);
                    }
                }

                robotPoses = newRobotPoses.ToArray();
                scan_Lines = newScanLines.ToArray();

            } // Filterout robot paths = true

            if (false) // Apply Poses Reduction
            {
                List<double[]> newRobotPoses = new List<double[]>();
                List<Matrix[]> newScanLines = new List<Matrix[]>();

                for (int i = 0; i < scan_Lines.Length; i += 4)
                {
                    if (scan_Lines[i].Length > 0)
                    {
                        newRobotPoses.Add(robotPoses[i]);
                        newScanLines.Add(scan_Lines[i]);
                    }
                }

                robotPoses = newRobotPoses.ToArray();
                scan_Lines = newScanLines.ToArray();
            } // Apply Poses Reduction => false

            if (true)
            {
                // Scan range middle point is X = 0, Z = 270
                double range = 7;
                for (int i = 0; i < scan_Lines.Length; i++)
                {

                    List<Matrix> newScanLine = new List<Matrix>();
                    for (int j = 0; j < scan_Lines[i].Length; j++)
                    {
                        if ((scan_Lines[i][j].Pz > 267 && 273 > scan_Lines[i][j].Pz) && (scan_Lines[i][j].Py > -range && range > scan_Lines[i][j].Py))
                        {
                            newScanLine.Add(scan_Lines[i][j]);
                        }
                    }

                    scan_Lines[i] = newScanLine.ToArray();
                }
            } // Keep only in distance range +/- 7mm

            if (true) // Clear Scanlines - Circle
            {
                for (int i = 0; i < scan_Lines.Length; i++)
                {
                    scan_Lines[i] = clearScanlinesInCircle(scan_Lines[i], ScanlineMinMaxClear); // 0.0001
                    //scan_Lines[i] = cleanScanlinesInPolynom2(scan_Lines[i], ScanlineMinMaxClear);
                }
            } // Clear Scanlines - Circle => true

            if (true) // Remove  without scanline data => true
            {
                List<double[]> newRobotPoses = new List<double[]>();
                List<Matrix[]> newScanLines = new List<Matrix[]>();

                for (int i = 0; i < scan_Lines.Length; i++)
                {
                    if (scan_Lines[i].Length > 0)
                    {
                        newRobotPoses.Add(robotPoses[i]);
                        newScanLines.Add(scan_Lines[i]);
                    }
                }

                robotPoses = newRobotPoses.ToArray();
                scan_Lines = newScanLines.ToArray();
            } // Remove  without scanline data => true

            if (true) // Apply Simple PointCloud Reduction
            {
                for (int i = 0; i < scan_Lines.Length; i++)
                {
                    scan_Lines[i] = simplePointCountReduction(scan_Lines[i], maxLinePointCount);
                }
            } // Apply Simple PointCloud Reduction => true

            if (false) // Apply Smooothing
            {
                for (int i = 0; i < scan_Lines.Length; i++)
                {
                    if (scan_Lines[i].Length < 100)
                    {
                        scan_Lines[i] = new Matrix[0];
                    }
                    else
                    {
                        scan_Lines[i] = scanlineSmooth(scan_Lines[i]);
                    }
                }
            } // Apply Smooothing => false

            //evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, TableDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance);

            return (robotPoses, scan_Lines);
        }

        #endregion

        #region Pre-Calibration
        (double stdErr, double[] resultDH) preCalibrationByTrig(double[][] robotPoses, Matrix[][] scan_Lines)
        {
            double increment = StartSearchIncr;
            //double incrDescent = IncrementDescent;

            double[] startDHTable = new double[TableDH.Length];
            TableDH.CopyTo(startDHTable, 0);

            double[] tableDH = new double[TableDH.Length];
            TableDH.CopyTo(tableDH, 0);

            double[][] calibPoses = new double[0][];
            double[][] calibCloudPoses = new double[0][];
            Matrix[] calibCenters = new Matrix[0];
            Matrix[][] calibCloud = new Matrix[0][];

            {
                List<double[]> poses = new List<double[]>();
                List<Matrix> centers = new List<Matrix>();

                int j = 0;
                int n = 0;
                List<double[]> subPoses = new List<double[]>();
                List<double[]> subCloudPoses = new List<double[]>();
                List<Matrix> subTrajectory = new List<Matrix>();
                List<Matrix[]> subScans = new List<Matrix[]>();
                List<Matrix[]> subCloud = new List<Matrix[]>();

                ProgresActionName = "Separating by scanner trigger ...";
                for (int i = 0; i < robotPoses.Length - 1; i++)
                {
                    subPoses.Add(robotPoses[i]);
                    subTrajectory.Add(extendedDHCalculations(tableDH, robotPoses[i]));
                    subScans.Add(scan_Lines[i]);

                    Matrix nextPoint = extendedDHCalculations(tableDH, robotPoses[i + 1]);
                    var dist = distance(subTrajectory[j].Px, subTrajectory[j].Py, subTrajectory[j].Pz, nextPoint.Px, nextPoint.Py, nextPoint.Pz);

                    if (dist > 10)
                    {
                        Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                        Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                        List<Matrix> pointCloud = new List<Matrix>();

                        for (int k = 0; k < subTrajectory.Count; k++)
                        {

                            pointCloud.AddRange(generate3DScanLine(subTrajectory[k], applyScannerMTX(Scanner_MTX, subScans[k])));
                        }
                        (Matrix sphereCenter, double radius) = getSphereCenter(pointCloud.ToArray());
                        centers.Add(sphereCenter);
                        poses.Add(subPoses[0]);

                        double qualityScore = (SurfaceTolerance / Math.Abs(ExpectedSphereRadius - radius));
                        if (qualityScore >= 1)
                        {
                            qualityScore = 1;
                        }

                        for (int k = 0; k < subScans.Count; k++)
                        {
                            Matrix[] reducedCloud = simplePointCountReduction(subScans[k], (int)(10 * qualityScore));
                            subCloud.Add(reducedCloud);
                        }

                        subCloudPoses.AddRange(subPoses);

                        subPoses = new List<double[]>();
                        subTrajectory = new List<Matrix>();
                        subScans = new List<Matrix[]>();
                        j = -1;
                    }

                    j++;

                    if (i > 0) { ProgressValue = (i / (float)robotPoses.Length) * 100; }
                }

                if (subPoses.Count > 0)
                {
                    Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                    Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                    List<Matrix> pointCloud = new List<Matrix>();

                    for (int k = 0; k < subTrajectory.Count; k++)
                    {

                        pointCloud.AddRange(generate3DScanLine(subTrajectory[k], applyScannerMTX(Scanner_MTX, subScans[k])));
                    }
                    (Matrix sphereCenter, double radius) = getSphereCenter(pointCloud.ToArray());
                    centers.Add(sphereCenter);
                    poses.Add(subPoses[0]);

                    double qualityScore = (SurfaceTolerance / Math.Abs(ExpectedSphereRadius - radius));
                    if (qualityScore >= 1)
                    {
                        qualityScore = 1;
                    }

                    for (int k = 0; k < subScans.Count; k++)
                    {
                        Matrix[] reducedCloud = simplePointCountReduction(subScans[k], (int)(25 * qualityScore));
                        subCloud.Add(reducedCloud);

                        //subCloud.Add(subScans[k]);
                    }

                    subCloudPoses.AddRange(subPoses);

                    subPoses = new List<double[]>();
                    subTrajectory = new List<Matrix>();
                    subScans = new List<Matrix[]>();
                }

                calibCloudPoses = subCloudPoses.ToArray();
                calibCloud = subCloud.ToArray();
                calibPoses = poses.ToArray();
                calibCenters = centers.ToArray();
            } // Get pure calib data

            {
                Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                for (int i = 0; i < calibPoses.Length; i++)
                {
                    Matrix robotPos = extendedDHCalculations(tableDH, calibPoses[i]);

                    robotPos = robotPos * Scanner_MTX;

                    Matrix centerOnFlange = robotPos.Inverse() * calibCenters[i];

                    calibCenters[i] = centerOnFlange;
                }
            } // Prepare calib data

            double oldError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, tableDH);
            BestError = oldError;

            int[] skipIndexes = DefaultSkipIndexes;
            double[] incr = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, 5, skipIndexes);

            double[] deltaIncrements = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, increment, skipIndexes);

            int repeatShake = 0;
            double[] winDH = new double[tableDH.Length];

            double[] bestDH = new double[tableDH.Length];
            tableDH.CopyTo(bestDH, 0);

            bool manualDHShift = false;
            IsEffective = true;
            ProgresActionName = "Precalibration in progress ...";
            while (IsEffective)
            {
                for (int i = 0; i < tableDH.Length; i++)
                {
                    tableDH[i] += (double)deltaIncrements[i];
                }

                double newError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, tableDH);

                if (newError < oldError)
                {
                    oldError = newError;
                    tableDH.CopyTo(bestDH, 0);

                    Task.Run(() => refreshGraph(startDHTable, bestDH));

                    CurrentIncrement = increment;
                    CurrentError = newError;

                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                    }
                }
                else
                {
                    bestDH.CopyTo(tableDH, 0);

                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                    }


                    increment *= IncDescentDiv;

                    CurrentIncrement = increment;
                }
                deltaIncrements = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, increment, skipIndexes);

                while (deltaIncrements.Min() == deltaIncrements.Max())
                {
                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                        break;
                    }

                    increment *= IncDescentDiv;
                    CurrentIncrement = increment;


                    deltaIncrements = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, increment, skipIndexes);

                }


                if (!IsEffective)
                {
                    if (manualDHShift)
                    {
                        skipIndexes = DefaultSkipIndexes;
                        manualDHShift = false;
                        increment = StartSearchIncr;
                        IsEffective = true;

                        if (newError < BestError)
                        {
                            BestError = newError;
                            tableDH.CopyTo(winDH, 0);

                            //trashHoldError = BestError * 10;
                        }

                        CurrentIncrement = increment;
                        CurrentError = BestError;

                        deltaIncrements = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, increment, skipIndexes);

                        while (deltaIncrements.Min() == deltaIncrements.Max())
                        {
                            if (increment < IncrementLimit)
                            {
                                IsEffective = false;
                                break;
                            }

                            increment *= IncDescentDiv;
                            CurrentIncrement = increment;


                            deltaIncrements = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, increment, skipIndexes);

                        }
                    }
                    else
                    {

                        if (newError < BestError)
                        {
                            BestError = newError;
                            tableDH.CopyTo(winDH, 0);

                            //trashHoldError = BestError * 10;
                        }
                        else
                        {
                            //if (Math.Abs(XReshakeOffset) > 5)
                            //{
                            //    break;
                            //}
                            break;
                        }

                        if (winDH.Sum() == 0)
                        {
                            tableDH.CopyTo(winDH, 0);
                        }

                        if (BestError <= StopError)
                        {
                            break;
                        }

                        TableDH.CopyTo(bestDH, 0);
                        CurrentIncrement = increment;
                        CurrentError = BestError;



                        repeatShake++;
                        tableDH[21] += XReshakeOffset;
                        //StartSearchIncr = Math.Abs(XReshakeOffset / 26);
                        //XReshakeOffset += 1;

                        newError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, tableDH);
                        CurrentError = newError;

                        if (newError < BestError)
                        {
                            BestError = newError;
                            tableDH.CopyTo(winDH, 0);
                        }


                        if (BestError > 10)
                        {
                            increment = BestError * 0.01;
                        }
                        else
                        {
                            increment = StartSearchIncr;
                        }


                        skipIndexes = new int[DefaultSkipIndexes.Length + 1];
                        Array.Copy(DefaultSkipIndexes, skipIndexes, DefaultSkipIndexes.Length);
                        skipIndexes[skipIndexes.Length - 1] = 21;
                        manualDHShift = true;

                        oldError = newError * 2;
                        IsEffective = true;
                        deltaIncrements = getDHIncrCenters(tableDH, calibPoses, calibCenters, calibCloud, calibCloudPoses, increment, skipIndexes);

                        ProgressValue = (StopError / BestError) * 100;
                    }
                }
            }


            double finalError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, winDH);
            CurrentError = finalError;

            return (finalError, winDH);
        }

        double evaluateCenters(double[][] robotPoses, Matrix[] centers, Matrix[][] calibCloud, double[][] calibCloudPoses, double[] inputDH)
        {
            double[] tableDH = new double[inputDH.Length];
            inputDH.CopyTo(tableDH, 0);


            Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
            Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

            /*
            double maxDist = 0;
            for (int i = 0; i < robotPoses.Length; i++)
            {
                Matrix centerInWorld = centersWorld[i];

                var dist = distance(centerPos[0], centerPos[1], centerPos[2], centerInWorld.Px, centerInWorld.Py, centerInWorld.Pz);

                if (dist > maxDist)
                {
                    maxDist = dist;
                }
            }
            */
            double min = double.MaxValue;
            double max = 0;
            double avgR = ExpectedSphereRadius;
            //List<double> onSurfaceDists = new List<double>();
            int allPoints = 0;
            for (int k = 0; k < calibCloudPoses.Length; k++)
            {
                Matrix robotPos = extendedDHCalculations(tableDH, calibCloudPoses[k]);
                Matrix[] lineCloud = generate3DScanLine(robotPos, applyScannerMTX(Scanner_MTX, calibCloud[k]));
                allPoints += lineCloud.Length;
                for (int l = 0; l < lineCloud.Length; l++)
                {
                    var dist = distance(ExpectedSpherePos[0], ExpectedSpherePos[1], ExpectedSpherePos[2], lineCloud[l].Px, lineCloud[l].Py, lineCloud[l].Pz);

                    if (dist < min) { min = dist; }
                    if (dist > max) { max = dist; }
                    avgR = (avgR + dist) / 2;

                    //onSurfaceDists.Add(dist);
                }
            }

            // Get average Radius = Average distance of all points from expected sphere center
            //int onSurfaceCount = 0;
            //foreach (double dist in onSurfaceDists)
            //{
            //
            //    if (dist < (avgR + SurfaceTolerance) && (avgR - SurfaceTolerance) < dist)
            //    {
            //        onSurfaceCount++;
            //    }
            //}

            double deltaRadius = Math.Abs(ExpectedSphereRadius - avgR);
            double minMax = max - min;
            //double radDelta = Math.Abs(ExpectedSphereRadius - avgR);
            //double coverage = (double)onSurfaceCount / allPoints;

            return minMax + deltaRadius;
        }

        double[] getDHIncrCenters(double[] tableDH, double[][] calibPoses, Matrix[] calibCenters, Matrix[][] calibCloud, double[][] calibCloudPoses, double increment, int[] skipIndexes)
        {
            double olrError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, tableDH);

            double[] tempDH = new double[tableDH.Length];
            tableDH.CopyTo(tempDH, 0);


            double[] deltaIncrements = new double[tempDH.Length];

            for (int i = 0; i < tempDH.Length; i++)
            {

                if (skipIndexes.Contains(i))
                {
                    deltaIncrements[i] = 0;
                    continue;
                }


                double origVal = tempDH[i];
                tempDH[i] += increment;

                double newError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, tempDH);

                if (newError < olrError)
                {
                    deltaIncrements[i] = olrError - newError;

                }
                else
                {
                    tempDH[i] -= 2 * increment;
                    newError = evaluateCenters(calibPoses, calibCenters, calibCloud, calibCloudPoses, tempDH);

                    if (newError < olrError)
                    {
                        deltaIncrements[i] = -(olrError - newError);

                    }
                }
                tempDH[i] = origVal;
            }

            // If found something usefull, set all values according gain... max gain => input increment value
            if (deltaIncrements.Min() != deltaIncrements.Max())
            {
                double div = increment / deltaIncrements.Select(x => Math.Abs(x)).Max();

                for (int i = 0; i < deltaIncrements.Length; i++)
                {
                    deltaIncrements[i] *= div;// * 0.1;
                }
            }

            return deltaIncrements;
        }

        (Matrix center, double radius) getSphereCenter(Matrix[] P)
        {
            double[,] a = new double[P.Length, 4];
            double[] f = new double[P.Length];

            int i = 0;
            foreach (Matrix mtx in P)
            {
                a[i, 0] = mtx.Px * 2;
                a[i, 1] = mtx.Py * 2;
                a[i, 2] = mtx.Pz * 2;
                a[i, 3] = 1.0f;
                f[i] = (mtx.Px * mtx.Px) + (mtx.Py * mtx.Py) + (mtx.Pz * mtx.Pz);
                i++;
            }

            var aMatrix = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.DenseOfArray(a);
            var fVector = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(f);

            var cVector = MultipleRegression.QR(aMatrix, fVector);

            // solve for the radius
            double radius = System.Math.Sqrt((double)((cVector[0] * cVector[0]) + (cVector[1] * cVector[1]) + (cVector[2] * cVector[2]) + cVector[3]));

            Matrix center = new Matrix();
            center.SetP(new Vector3(cVector[0], cVector[1], cVector[2]));
            return (center, radius);
        }

        #endregion Pre-Calibration

        #region Path optimisation
        Matrix[] extractBoundary(Matrix[] sharpCloud, double boundDist = 2.5)
        {
            List<Matrix> boundary = new List<Matrix>();

            double dist = 0;
            for (int i = 0; i < sharpCloud.Length - 1; i++)
            {
                dist = distance(sharpCloud[i].Px, sharpCloud[i].Py, sharpCloud[i].Pz, sharpCloud[i + 1].Px, sharpCloud[i + 1].Py, sharpCloud[i + 1].Pz);
                if (dist >= boundDist)
                {
                    boundary.Add(sharpCloud[i]);
                    boundary.Add(sharpCloud[i + 1]);
                    i++;
                }

            }

            return boundary.ToArray();
        }

        Matrix[] pathOptimiser(Matrix[] sharpCloud, double[] timeStamps, double cutOut, int cutEndsLength)
        {


            Matrix[] filteredCloud = filterMotionCloud(sharpCloud, 2, cutOut, cutEndsLength);

            //return filteredCloud;

            Matrix[] interpolatedCloud = interpolateMotionCloud(filteredCloud);

            return interpolatedCloud;

            //Matrix[] smoothCloud = smoothMotionCloud(interpolatedCloud, timeStamps, sharpCloud.Length / 30);
            //
            //return smoothCloud;
        }

        Matrix[] filterMotionCloud(Matrix[] sharpCloud, int orderVal, double cutOutRatio, int cutEndsLength)
        {
            // P
            double[] vals_Px = new double[sharpCloud.Length];
            double[] vals_Py = new double[sharpCloud.Length];
            double[] vals_Pz = new double[sharpCloud.Length];

            double[] polynomXvals = new double[sharpCloud.Length];



            for (int i = 0; i < sharpCloud.Length; i++)
            {
                // P
                vals_Px[i] = sharpCloud[i].Px;
                vals_Py[i] = sharpCloud[i].Py;
                vals_Pz[i] = sharpCloud[i].Pz;

                polynomXvals[i] = i;

            }

            DirectRegressionMethod regMethod = DirectRegressionMethod.QR;

            // P
            var func_Px = MathNet.Numerics.Polynomial.Fit(polynomXvals, vals_Px, orderVal, regMethod);
            var func_Py = MathNet.Numerics.Polynomial.Fit(polynomXvals, vals_Py, orderVal, regMethod);
            var func_Pz = MathNet.Numerics.Polynomial.Fit(polynomXvals, vals_Pz, orderVal, regMethod);


            // get FIRST polynom point 
            double firstPoly = polynomXvals[0];
            double firstX = MathNet.Numerics.Polynomial.Evaluate(firstPoly, func_Px.Coefficients);
            double firstY = MathNet.Numerics.Polynomial.Evaluate(firstPoly, func_Py.Coefficients);
            double firstZ = MathNet.Numerics.Polynomial.Evaluate(firstPoly, func_Pz.Coefficients);

            // get LAST polynom point 
            double lastPoly = polynomXvals.Last();
            double lastX = MathNet.Numerics.Polynomial.Evaluate(lastPoly, func_Px.Coefficients);
            double lastY = MathNet.Numerics.Polynomial.Evaluate(lastPoly, func_Py.Coefficients);
            double lastZ = MathNet.Numerics.Polynomial.Evaluate(lastPoly, func_Pz.Coefficients);

            Vector3 polyVec = new Vector3(lastX - firstX, lastY - firstY, lastZ - firstZ);

            double[] distances = new double[sharpCloud.Length];
            Dictionary<int, double> indexDist = new Dictionary<int, double>();
            for (int i = 0; i < sharpCloud.Length; i++)
            {
                Vector3 pointVec = new Vector3(sharpCloud[i].Px - firstX, sharpCloud[i].Py - firstY, sharpCloud[i].Pz - firstZ);

                var shortestDist = pointVec.Length * Math.Sin(Vector3.AngleBetween(polyVec, pointVec));

                indexDist[i] = shortestDist;
                distances[i] = shortestDist;
            }
            Array.Sort(distances);

            // Remove last 20%
            int inCount = (int)(sharpCloud.Length * (1 - cutOutRatio));
            int maxHole = 5; // Max hole in milimeters

            distances = distances.Take(inCount).ToArray();

            double[] lastGoodPos = new double[3];

            Matrix[] filteredCloud = new Matrix[sharpCloud.Length];
            for (int i = 0; i < sharpCloud.Length; i++)
            {
                var shortestDist = indexDist[i];

                Matrix newMTX = new Matrix();

                if (i > cutEndsLength && i < sharpCloud.Length - cutEndsLength)
                {
                    if (distances.Contains(shortestDist))
                    {
                        newMTX = sharpCloud[i];
                        lastGoodPos[0] = newMTX.Px;
                        lastGoodPos[1] = newMTX.Py;
                        lastGoodPos[2] = newMTX.Pz;
                    }
                    else
                    {
                        double dist = distance(lastGoodPos[0], lastGoodPos[1], lastGoodPos[2], sharpCloud[i].Px, sharpCloud[i].Py, sharpCloud[i].Pz);
                        if (dist >= maxHole)
                        {
                            newMTX = sharpCloud[i];
                            lastGoodPos[0] = newMTX.Px;
                            lastGoodPos[1] = newMTX.Py;
                            lastGoodPos[2] = newMTX.Pz;
                        }
                    }
                }

                filteredCloud[i] = newMTX;
            }

            return filteredCloud;
        }

        Matrix[] interpolateMotionCloud(Matrix[] sharpCloud)
        {
            // P
            double[] vals_Px = new double[sharpCloud.Length];
            double[] vals_Py = new double[sharpCloud.Length];
            double[] vals_Pz = new double[sharpCloud.Length];

            // N
            double[] vals_Nx = new double[sharpCloud.Length];
            double[] vals_Ny = new double[sharpCloud.Length];
            double[] vals_Nz = new double[sharpCloud.Length];

            // O
            double[] vals_Ox = new double[sharpCloud.Length];
            double[] vals_Oy = new double[sharpCloud.Length];
            double[] vals_Oz = new double[sharpCloud.Length];

            // A
            double[] vals_Ax = new double[sharpCloud.Length];
            double[] vals_Ay = new double[sharpCloud.Length];
            double[] vals_Az = new double[sharpCloud.Length];

            double[] polyXvals = new double[sharpCloud.Length];

            bool avgPosRequest = false;
            int lastNonZeroPos = 0;
            for (int i = 0; i < sharpCloud.Length; i++)
            {
                if (sharpCloud[i].Pz != 0)
                {
                    // P
                    vals_Px[i] = sharpCloud[i].Px;
                    vals_Py[i] = sharpCloud[i].Py;
                    vals_Pz[i] = sharpCloud[i].Pz;

                    // N
                    vals_Nx[i] = sharpCloud[i].Nx;
                    vals_Ny[i] = sharpCloud[i].Ny;
                    vals_Nz[i] = sharpCloud[i].Nz;

                    // O
                    vals_Ox[i] = sharpCloud[i].Ox;
                    vals_Oy[i] = sharpCloud[i].Oy;
                    vals_Oz[i] = sharpCloud[i].Oz;

                    // A
                    vals_Ax[i] = sharpCloud[i].Ax;
                    vals_Ay[i] = sharpCloud[i].Ay;
                    vals_Az[i] = sharpCloud[i].Az;

                    polyXvals[i] = i;

                    if (avgPosRequest && lastNonZeroPos > 0)
                    {
                        avgPosRequest = false;
                    
                        int diffIndex = i - lastNonZeroPos;
                    
                        // P
                        double diffPx = (sharpCloud[i].Px - vals_Px[lastNonZeroPos]) / diffIndex;
                        double diffPy = (sharpCloud[i].Py - vals_Py[lastNonZeroPos]) / diffIndex;
                        double diffPz = (sharpCloud[i].Pz - vals_Pz[lastNonZeroPos]) / diffIndex;
                    
                        // N
                        double diffNx = (sharpCloud[i].Nx - vals_Nx[lastNonZeroPos]) / diffIndex;
                        double diffNy = (sharpCloud[i].Ny - vals_Ny[lastNonZeroPos]) / diffIndex;
                        double diffNz = (sharpCloud[i].Nz - vals_Nz[lastNonZeroPos]) / diffIndex;
                    
                        // O
                        double diffOx = (sharpCloud[i].Ox - vals_Ox[lastNonZeroPos]) / diffIndex;
                        double diffOy = (sharpCloud[i].Oy - vals_Oy[lastNonZeroPos]) / diffIndex;
                        double diffOz = (sharpCloud[i].Oz - vals_Oz[lastNonZeroPos]) / diffIndex;
                    
                        // A
                        double diffAx = (sharpCloud[i].Ax - vals_Ax[lastNonZeroPos]) / diffIndex;
                        double diffAy = (sharpCloud[i].Ay - vals_Ay[lastNonZeroPos]) / diffIndex;
                        double diffAz = (sharpCloud[i].Az - vals_Az[lastNonZeroPos]) / diffIndex;
                    
                    
                        for (int j = lastNonZeroPos + 1; j < i; j++)
                        {
                            // P
                            vals_Px[j] = vals_Px[j - 1] + diffPx;
                            vals_Py[j] = vals_Py[j - 1] + diffPy;
                            vals_Pz[j] = vals_Pz[j - 1] + diffPz;
                    
                            // N
                            vals_Nx[j] = vals_Nx[j - 1] + diffNx;
                            vals_Ny[j] = vals_Ny[j - 1] + diffNy;
                            vals_Nz[j] = vals_Nz[j - 1] + diffNz;
                    
                            // O
                            vals_Ox[j] = vals_Ox[j - 1] + diffOx;
                            vals_Oy[j] = vals_Oy[j - 1] + diffOy;
                            vals_Oz[j] = vals_Oz[j - 1] + diffOz;
                    
                            // A
                            vals_Ax[j] = vals_Ax[j - 1] + diffAx;
                            vals_Ay[j] = vals_Ay[j - 1] + diffAy;
                            vals_Az[j] = vals_Az[j - 1] + diffAz;
                        }
                    }

                    lastNonZeroPos = i;
                }
                else
                {
                    avgPosRequest = true;
                }
            }

            Matrix[] interpolatedCloud = new Matrix[sharpCloud.Length];

            int k = 0;
            for (int xVal = 0; xVal < sharpCloud.Length; xVal++)
            {

                Matrix newMTX = new Matrix();

                // P
                newMTX.Px = vals_Px[xVal];
                newMTX.Py = vals_Py[xVal];
                newMTX.Pz = vals_Pz[xVal];

                // N
                newMTX.Nx = vals_Nx[xVal];
                newMTX.Ny = vals_Ny[xVal];
                newMTX.Nz = vals_Nz[xVal];

                // O
                newMTX.Ox = vals_Ox[xVal];
                newMTX.Oy = vals_Oy[xVal];
                newMTX.Oz = vals_Oz[xVal];

                // A
                newMTX.Ax = vals_Ax[xVal];
                newMTX.Ay = vals_Ay[xVal];
                newMTX.Az = vals_Az[xVal];

                interpolatedCloud[k] = newMTX;
                k++;
            }

            return interpolatedCloud;
        }

        Matrix[] smoothMotionCloud(Matrix[] sharpCloud, double[] timeStamps, int orderVal)
        {
            // P
            double[] vals_Px = new double[sharpCloud.Length];
            double[] vals_Py = new double[sharpCloud.Length];
            double[] vals_Pz = new double[sharpCloud.Length];

            // N
            double[] vals_Nx = new double[sharpCloud.Length];
            double[] vals_Ny = new double[sharpCloud.Length];
            double[] vals_Nz = new double[sharpCloud.Length];

            // O
            double[] vals_Ox = new double[sharpCloud.Length];
            double[] vals_Oy = new double[sharpCloud.Length];
            double[] vals_Oz = new double[sharpCloud.Length];

            // A
            double[] vals_Ax = new double[sharpCloud.Length];
            double[] vals_Ay = new double[sharpCloud.Length];
            double[] vals_Az = new double[sharpCloud.Length];

            double[] polyXvals = new double[sharpCloud.Length];

            //double timeCutOff = timeStamps[0] - (timeStamps.Last() - timeStamps[0]);
            double timeCutOff = timeStamps.Last() - timeStamps[0];

            for (int i = 0; i < sharpCloud.Length; i++)
            {
                if (sharpCloud[i].Pz != 0)
                {
                    // P
                    vals_Px[i] = sharpCloud[i].Px;
                    vals_Py[i] = sharpCloud[i].Py;
                    vals_Pz[i] = sharpCloud[i].Pz;

                    // N
                    vals_Nx[i] = sharpCloud[i].Nx;
                    vals_Ny[i] = sharpCloud[i].Ny;
                    vals_Nz[i] = sharpCloud[i].Nz;

                    // O
                    vals_Ox[i] = sharpCloud[i].Ox;
                    vals_Oy[i] = sharpCloud[i].Oy;
                    vals_Oz[i] = sharpCloud[i].Oz;

                    // A
                    vals_Ax[i] = sharpCloud[i].Ax;
                    vals_Ay[i] = sharpCloud[i].Ay;
                    vals_Az[i] = sharpCloud[i].Az;

                    polyXvals[i] = timeStamps[i] - timeCutOff;
                }
            }

            DirectRegressionMethod regMethod = DirectRegressionMethod.QR;

            // P
            var func_Px = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Px, orderVal, regMethod);
            var func_Py = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Py, orderVal, regMethod);
            var func_Pz = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Pz, orderVal, regMethod);

            // N
            var func_Nx = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Nx, orderVal, regMethod);
            var func_Ny = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Ny, orderVal, regMethod);
            var func_Nz = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Nz, orderVal, regMethod);

            // O
            var func_Ox = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Ox, orderVal, regMethod);
            var func_Oy = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Oy, orderVal, regMethod);
            var func_Oz = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Oz, orderVal, regMethod);

            // A
            var func_Ax = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Ax, orderVal, regMethod);
            var func_Ay = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Ay, orderVal, regMethod);
            var func_Az = MathNet.Numerics.Polynomial.Fit(polyXvals, vals_Az, orderVal, regMethod);

            Matrix[] smoothCloud = new Matrix[sharpCloud.Length];

            int k = 0;
            foreach (double xVal in timeStamps)
            {
                Matrix newMTX = new Matrix();

                // P
                newMTX.Px = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Px.Coefficients);
                newMTX.Py = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Py.Coefficients);
                newMTX.Pz = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Pz.Coefficients);

                // N
                newMTX.Nx = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Nx.Coefficients);
                newMTX.Ny = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Ny.Coefficients);
                newMTX.Nz = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Nz.Coefficients);

                // O
                newMTX.Ox = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Ox.Coefficients);
                newMTX.Oy = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Oy.Coefficients);
                newMTX.Oz = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Oz.Coefficients);

                // A
                newMTX.Ax = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Ax.Coefficients);
                newMTX.Ay = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Ay.Coefficients);
                newMTX.Az = MathNet.Numerics.Polynomial.Evaluate(xVal - timeCutOff, func_Az.Coefficients);

                smoothCloud[k] = newMTX;
                k++;
            }

            return smoothCloud;
        }

        #endregion

        #region Calibration functions

        private (double stdErr, double[] resultDH) singleSphereCalibration(double[][] robotPoses, Matrix[][] scan_Lines)
        {
            double increment = StartSearchIncr;

            double[] startDHTable = new double[TableDH.Length];
            TableDH.CopyTo(startDHTable, 0);

            double[] tableDH = new double[TableDH.Length];
            TableDH.CopyTo(tableDH, 0);


            double oldError = evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, tableDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;
            if (oldError == -1)
            {
                return (-1, new double[0]);
            }
            BestError = oldError;

            SingleSphere_3DShow(generate3DPointCloud(robotPoses, scan_Lines, tableDH), true);

            int[] skipIndexes = DefaultSkipIndexes;
            double[] deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes);

            int repeatShake = 0;
            double[] winDH = new double[tableDH.Length];

            double[] bestDH = new double[tableDH.Length];
            tableDH.CopyTo(bestDH, 0);

            double lastFrozenIncrement = 0;
            int frozenRepeats = 0;

            bool manualDHShift = false;
            IsEffective = true;
            while (IsEffective)
            {
                for (int i = 0; i < tableDH.Length; i++)
                {
                    tableDH[i] += (double)deltaIncrements[i];
                }

                double newError = evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, tableDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;


                if (newError < oldError)
                {

                    oldError = newError;
                    tableDH.CopyTo(bestDH, 0);


                    CurrentIncrement = increment;
                    CurrentError = newError;

                    Task.Run(() => SingleSphere_3DShow(generate3DPointCloud(robotPoses, scan_Lines, tableDH), true));
                    Task.Run(() => refreshGraph(startDHTable, bestDH));

                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                    }
                }
                else
                {
                    bestDH.CopyTo(tableDH, 0);

                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                    }


                    increment *= IncDescentDiv;

                    CurrentIncrement = increment;

                    deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes); // changed 2023-09-05
                }
                //deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes); // changed 2023-09-05

                while (deltaIncrements.Min() == deltaIncrements.Max())
                {
                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                        break;
                    }

                    increment *= IncDescentDiv;
                    CurrentIncrement = increment;


                    deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes);

                }


                if (!IsEffective)
                {
                    if (manualDHShift)
                    {
                        tableDH[19] -= XReshakeOffset;

                        skipIndexes = DefaultSkipIndexes;
                        manualDHShift = false;
                        increment = StartSearchIncr;
                        IsEffective = true;

                        if (oldError < BestError)
                        {
                            BestError = oldError;
                            bestDH.CopyTo(winDH, 0);
                        }

                        CurrentIncrement = increment;

                        deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes);

                        while (deltaIncrements.Min() == deltaIncrements.Max())
                        {
                            if (increment < IncrementLimit)
                            {
                                IsEffective = false;
                                break;
                            }

                            increment *= IncDescentDiv;
                            CurrentIncrement = increment;


                            deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes);

                        }
                    }
                    else
                    {
                        if (oldError < BestError)
                        {
                            BestError = oldError;
                            bestDH.CopyTo(winDH, 0);
                        }
                        else
                        {
                            if (repeatShake > 0)
                            {
                                break;
                            }
                        }

                        if (winDH.Sum() == 0)
                        {
                            tableDH.CopyTo(winDH, 0);
                        }

                        if (BestError <= StopError)
                        {
                            break;
                        }

                        winDH.CopyTo(bestDH, 0);
                        CurrentIncrement = increment;
                        CurrentError = BestError;



                        repeatShake++;
                        tableDH[19] += XReshakeOffset;

                        newError = evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, tableDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;
                        CurrentError = newError;

                        if (newError < BestError)
                        {
                            BestError = newError;
                            tableDH.CopyTo(winDH, 0);
                        }


                        if (BestError > 10)
                        {
                            increment = BestError * 0.01;
                        }
                        else
                        {
                            increment = StartSearchIncr;
                        }


                        skipIndexes = new int[DefaultSkipIndexes.Length + 1];
                        Array.Copy(DefaultSkipIndexes, skipIndexes, DefaultSkipIndexes.Length);
                        skipIndexes[skipIndexes.Length - 1] = 21;
                        manualDHShift = true;

                        oldError = newError * 2;
                        IsEffective = true;
                        deltaIncrements = getDHIncrements(tableDH, robotPoses, scan_Lines, increment, skipIndexes);

                        ProgressValue = (StopError / BestError) * 100;
                    }
                }
            }


            double finalError = evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, winDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;
            CurrentError = finalError;

            return (finalError, winDH);
        }

        private (double stdErr, double[] resultDH) multiSphereCalibration(List<CalibSphere> toEvalSpheres)
        {
            StopError *= toEvalSpheres.Count;

            double increment = StartSearchIncr;
            //double incrDescent = IncrementDescent;

            double[] startDHTable = new double[TableDH.Length];
            TableDH.CopyTo(startDHTable, 0);

            double[] tableDH = new double[TableDH.Length];
            TableDH.CopyTo(tableDH, 0);


            double oldError = evaluateMultiSphereData(toEvalSpheres, tableDH).error;
            if (oldError == -1)
            {
                return (-1, new double[0]);
            }
            BestError = oldError;
            //double trashHoldError = oldError*2;

            MultiSphere_3DShow(toEvalSpheres, tableDH);

            int[] skipIndexes = DefaultSkipIndexes;
            double[] deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes);

            int repeatShake = 0;
            double[] winDH = new double[tableDH.Length];

            double[] bestDH = new double[tableDH.Length];
            tableDH.CopyTo(bestDH, 0);

            bool manualDHShift = false;
            IsEffective = true;
            while (IsEffective)
            {
                for (int i = 0; i < tableDH.Length; i++)
                {
                    tableDH[i] += (double)deltaIncrements[i];
                }

                double newError = evaluateMultiSphereData(toEvalSpheres, tableDH).error;

                if (newError < oldError)
                {
                    oldError = newError;
                    tableDH.CopyTo(bestDH, 0);

                    CurrentIncrement = increment;
                    CurrentError = newError;

                    Task.Run(() => MultiSphere_3DShow(toEvalSpheres, tableDH));
                    Task.Run(() => refreshGraph(startDHTable, bestDH));


                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                    }
                }
                else
                {
                    bestDH.CopyTo(tableDH, 0);

                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                    }


                    increment *= IncDescentDiv;

                    CurrentIncrement = increment;
                    
                    deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes); // changed 2023-09-05
                }
                //deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes); // changed 2023-09-05

                while (deltaIncrements.Min() == deltaIncrements.Max())
                {
                    if (increment < IncrementLimit)
                    {
                        IsEffective = false;
                        break;
                    }

                    increment *= IncDescentDiv;
                    CurrentIncrement = increment;


                    deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes);

                }


                if (!IsEffective)
                {
                    if (manualDHShift)
                    {
                        tableDH[19] -= XReshakeOffset;

                        skipIndexes = DefaultSkipIndexes;
                        manualDHShift = false;
                        increment = StartSearchIncr;
                        IsEffective = true;

                        if (oldError < BestError)
                        {
                            BestError = oldError;
                            bestDH.CopyTo(winDH, 0);
                        }

                        CurrentIncrement = increment;

                        deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes);

                        while (deltaIncrements.Min() == deltaIncrements.Max())
                        {
                            if (increment < IncrementLimit)
                            {
                                IsEffective = false;
                                break;
                            }

                            increment *= IncDescentDiv;
                            CurrentIncrement = increment;


                            deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes);

                        }
                    }
                    else
                    {
                        if (oldError < BestError)
                        {
                            BestError = oldError;
                            bestDH.CopyTo(winDH, 0);
                        }
                        else
                        {
                            if (repeatShake > 0)
                            {
                                break;
                            }
                            //break;
                        }

                        if (winDH.Sum() == 0)
                        {
                            tableDH.CopyTo(winDH, 0);
                        }

                        if (BestError <= StopError)
                        {
                            break;
                        }

                        winDH.CopyTo(bestDH, 0);
                        CurrentIncrement = increment;
                        CurrentError = BestError;



                        repeatShake++;
                        tableDH[19] += XReshakeOffset;

                        newError = evaluateMultiSphereData(toEvalSpheres, tableDH).error;
                        CurrentError = newError;

                        if (newError < BestError)
                        {
                            BestError = newError;
                            tableDH.CopyTo(winDH, 0);
                        }


                        if (BestError > 10)
                        {
                            increment = BestError * 0.01;
                        }
                        else
                        {
                            increment = StartSearchIncr;
                        }


                        skipIndexes = new int[DefaultSkipIndexes.Length + 1];
                        Array.Copy(DefaultSkipIndexes, skipIndexes, DefaultSkipIndexes.Length);
                        skipIndexes[skipIndexes.Length - 1] = 21;
                        manualDHShift = true;

                        oldError = newError * 2;
                        IsEffective = true;
                        deltaIncrements = getDHIncrementsMulti(tableDH, toEvalSpheres, increment, skipIndexes);

                        ProgressValue = (StopError / BestError) * 100;
                    }
                }
            }

            double finalError = evaluateMultiSphereData(toEvalSpheres, winDH).error;
            CurrentError = finalError;

            StopError /= toEvalSpheres.Count;

            return (finalError, winDH);
        }

        (double surfaceCoverage, double error) evaluateMultiSphereData(List<CalibSphere> toEvalSpheres, double[] tableDH)
        {
            double avgCoverage = 0;
            double avgError = 0;

            List<Task<(double radius, double surfaceCoverage, double error)>> tasks = new List<Task<(double radius, double surfaceCoverage, double error)>>();
            foreach (var sphere in toEvalSpheres)
            {
                tasks.Add(Task<(double radius, double surfaceCoverage, double error)>.Factory.StartNew(() => 
                evaluateSingleSphereData(generate3DPointCloud(sphere.robotPoses, sphere.scan_Lines, tableDH), sphere.expectedPosition, sphere.expectedRadius, sphere.surfaceTolerance)));
            }

            Task.WaitAll(tasks.ToArray());
            foreach (var task in tasks)
            {
                avgCoverage += task.Result.surfaceCoverage;
                avgError += task.Result.error;
            }

            return (avgCoverage, avgError);
        }

        (double radius, double surfaceCoverage, double error) evaluateSingleSphereData(Matrix[] P, double[] expectedPosition, double expectedRadius, double surfaceTolerance)
        {
            double[,] a = new double[P.Length, 4];
            double[] f = new double[P.Length];

            int i = 0;
            foreach (Matrix mtx in P)
            {
                a[i, 0] = mtx.Px * 2;
                a[i, 1] = mtx.Py * 2;
                a[i, 2] = mtx.Pz * 2;
                a[i, 3] = 1.0f;
                f[i] = (mtx.Px * mtx.Px) + (mtx.Py * mtx.Py) + (mtx.Pz * mtx.Pz);
                i++;
            }

            var aMatrix = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.DenseOfArray(a);
            var fVector = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(f);

            var cVector = MultipleRegression.QR(aMatrix, fVector);

            // solve for the radius
            double radius = System.Math.Sqrt((double)((cVector[0] * cVector[0]) + (cVector[1] * cVector[1]) + (cVector[2] * cVector[2]) + cVector[3]));

            double maxRadius = 0;
            double minRadius = double.MaxValue;
            double onSurfaceCount = 0;

            foreach (Matrix point in P)
            {
                double dist = distance(cVector[0], cVector[1], cVector[2], (double)point.Px, (double)point.Py, (double)point.Pz);

                if (dist > maxRadius)
                {
                    maxRadius = dist;
                }

                if (dist < minRadius)
                {
                    minRadius = dist;
                }

                if (dist < (radius + surfaceTolerance) && (radius - surfaceTolerance) < dist)
                {
                    onSurfaceCount++;
                }
            }


            double coverage = onSurfaceCount / P.Length;

            var radiusErr = Math.Abs(radius - expectedRadius);
            var minMax = Math.Abs(maxRadius - minRadius);
            //
            var error = minMax + radiusErr;
            error /= coverage;


            if (expectedPosition != null)
            {
                var dist = distance(cVector[0], cVector[1], cVector[2], expectedPosition[0], expectedPosition[1], expectedPosition[2]);
                error += dist;
            }

            return (radius, coverage, error);
        }

        double[] getDHIncrements(double[] tableDH, double[][] robotPoses, Matrix[][] scan_Lines, double increment, int[] skipIndexes)
        {
            Matrix[] pointCloud = generate3DPointCloud(robotPoses, scan_Lines, tableDH);
            double oldError = evaluateSingleSphereData(pointCloud, ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;

            double[] tempDH = new double[tableDH.Length];
            tableDH.CopyTo(tempDH, 0);


            double[] deltaIncrements = new double[tempDH.Length];

            for (int i = 0; i < tempDH.Length; i++)
            {

                if (skipIndexes.Contains(i))
                {
                    deltaIncrements[i] = 0;
                    continue;
                }


                double origVal = tempDH[i];
                tempDH[i] += increment;

                double newError = evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, tempDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;

                if (newError < oldError)
                {
                    deltaIncrements[i] = Math.Pow(oldError - newError, 2);
                }
                else
                {
                    tempDH[i] -= 2 * increment;
                    newError = evaluateSingleSphereData(generate3DPointCloud(robotPoses, scan_Lines, tempDH), ExpectedSpherePos, ExpectedSphereRadius, SurfaceTolerance).error;

                    if (newError < oldError)
                    {
                        deltaIncrements[i] = -(Math.Pow(oldError - newError, 2));
                    }
                }
                tempDH[i] = origVal;
            }

            // If found something usefull, set all values according gain... max gain => input increment value
            if (deltaIncrements.Min() != deltaIncrements.Max())
            {
                double div = increment / deltaIncrements.Select(x => Math.Abs(x)).Max();
            
                for (int i = 0; i < deltaIncrements.Length; i++)
                {
                    deltaIncrements[i] *= div;// * 0.1;
                }
            }

            
            return deltaIncrements;
        }

        double[] getDHIncrementsMulti(double[] tableDH, List<CalibSphere> toEvalSpheres, double increment, int[] skipIndexes)
        {

            // Prerobiť na paralelné vyhodnocovanie
            double oldError = evaluateMultiSphereData(toEvalSpheres, tableDH).error;

            double[] tempDH = new double[tableDH.Length];
            tableDH.CopyTo(tempDH, 0);


            double[] deltaIncrements = new double[tempDH.Length];


            for (int i = 0; i < tempDH.Length -1; i++)
            {
                if (skipIndexes.Contains(i))
                {
                    deltaIncrements[i] = 0;
                    continue;
                }

                double origVal = tempDH[i];
                tempDH[i] += increment;
                
                double newError = evaluateMultiSphereData(toEvalSpheres, tempDH).error;
                
                if (newError < oldError)
                {
                    deltaIncrements[i] = Math.Pow(oldError - newError, 2);
                }
                else
                {
                    tempDH[i] -= 2 * increment;
                    newError = evaluateMultiSphereData(toEvalSpheres, tempDH).error;
                
                    if (newError < oldError)
                    {
                        deltaIncrements[i] = -(Math.Pow(oldError - newError, 2));
                    }
                }
                tempDH[i] = origVal;
            }

            // If found something usefull, set all values according gain... max gain => input increment value
            if (deltaIncrements.Min() != deltaIncrements.Max())
            {
                double div = increment / deltaIncrements.Select(x => Math.Abs(x)).Max();
            
                for (int i = 0; i < deltaIncrements.Length; i++)
                {
                    deltaIncrements[i] *= div;// * 0.1;
                }
            }


            return deltaIncrements;
        }
                
        #endregion

        #region Base functions

        Matrix[] generate3DPointCloud(double[][] robotPoses, Matrix[][] scan_Lines, double[] tableDH)
        {
            Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
            Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

            Matrix pulledScanner = Scanner_MTX;
            Matrix pushedScanner = Scanner_MTX;

            pulledScanner.Px += tableDH[34];
            pushedScanner.Px -= tableDH[34];

            pulledScanner.Py += tableDH[35];
            pushedScanner.Py -= tableDH[35];

            pulledScanner.Pz += tableDH[36];
            pushedScanner.Pz -= tableDH[36];

            List<Matrix> pointCloud = new List<Matrix>();
            Matrix previus_MTX = extendedDHCalculations(tableDH, robotPoses[0]);

            for (int i = 0; i < robotPoses.Length; i++)
            {
                Matrix robot_MTX = extendedDHCalculations(tableDH, robotPoses[i]);

                Vector3 deltaPos = previus_MTX.GetP() - robot_MTX.GetP();
                previus_MTX = robot_MTX;
                if (deltaPos.Z < 0)
                {
                    pointCloud.AddRange(generate3DScanLine(robot_MTX, applyScannerMTX(pulledScanner, scan_Lines[i])));
                }
                else
                {
                    pointCloud.AddRange(generate3DScanLine(robot_MTX, applyScannerMTX(pushedScanner, scan_Lines[i])));
                }
                
                //pointCloud.AddRange(generate3DScanLine(robot_MTX, applyScannerMTX(Scanner_MTX, scan_Lines[i])));

            }

            return pointCloud.ToArray();
        }

        Matrix[] generate3DScanLine(Matrix robot_MTX, Matrix[] scan_Points)
        {
            Matrix[] pointCloud = new Matrix[scan_Points.Length];

            for (int i = 0; i < scan_Points.Length; i++)
            {
                Matrix newMTX = robot_MTX;
                newMTX.TranslateRelative(scan_Points[i].GetP());
                pointCloud[i] = newMTX;
            }

            return pointCloud;
        }

        Matrix[] applyScannerMTX(Matrix offsetMatrix, Matrix[] scan_Points)
        {
            Matrix[] shiftedScanLines = new Matrix[scan_Points.Length];

            for (int i = 0; i < scan_Points.Length; i++)
            {
                shiftedScanLines[i] = Matrix.Multiply(offsetMatrix, scan_Points[i]);
            }

            return shiftedScanLines;
        }

        private Matrix OLD_myDHCalculations(double[] tableDH, double[] jointValues)
        {
            //Matrix T0 = new Matrix();
            Matrix[] T = new Matrix[7];

            { // Sin(0) = 0    ||  Cos(0) = 1
                T[0].Nx = Math.Cos(-jointValues[1] * DegToRad);
                T[0].Ny = Math.Sin(-jointValues[1] * DegToRad);
                T[0].Nz = 0;
                T[0].Nw = 0;

                T[0].Ox = Math.Sin(-jointValues[1] * DegToRad);
                T[0].Oy = -Math.Cos(-jointValues[1] * DegToRad);
                T[0].Oz = 0;
                T[0].Ow = 0;

                T[0].Ax = 0;
                T[0].Ay = 0;
                T[0].Az = -1;
                T[0].Aw = 0;

                T[0].Px = 0;
                T[0].Py = 0;
                T[0].Pz = 0;// tableDH[2];// <== Nemá mať efekt na guľu, mení Z v absolútnom smere - Vhodné na presun do Py, keďže to jediné nieje pohyblivé s aktuálnou DH tabuľkou
                T[0].Pw = 1;
            } // T0

            {
                T[1].Nx = 1;
                T[1].Ny = 0;
                T[1].Nz = 0;
                T[1].Nw = 0;

                T[1].Ox = 0;
                T[1].Oy = Math.Cos(tableDH[1] * DegToRad);
                T[1].Oz = Math.Sin(tableDH[1] * DegToRad);
                T[1].Ow = 0;

                T[1].Ax = 0;
                T[1].Ay = -Math.Sin(tableDH[1] * DegToRad);
                T[1].Az = Math.Cos(tableDH[1] * DegToRad);
                T[1].Aw = 0;

                T[1].Px = tableDH[0];
                T[1].Py = -tableDH[5] * Math.Sin(tableDH[1] * DegToRad);
                T[1].Pz = tableDH[5] * Math.Cos(tableDH[1] * DegToRad);
                T[1].Pw = 1;
            } // T1

            {
                T[2].Nx = Math.Cos(jointValues[2] * DegToRad);
                T[2].Ny = Math.Cos(tableDH[4] * DegToRad) * Math.Sin(jointValues[2] * DegToRad);
                T[2].Nz = Math.Sin(tableDH[4] * DegToRad) * Math.Sin(jointValues[2] * DegToRad);
                T[2].Nw = 0;

                T[2].Ox = -Math.Sin(jointValues[2] * DegToRad);
                T[2].Oy = Math.Cos(tableDH[4] * DegToRad) * Math.Cos(jointValues[2] * DegToRad);
                T[2].Oz = Math.Sin(tableDH[4] * DegToRad) * Math.Cos(jointValues[2] * DegToRad);
                T[2].Ow = 0;

                T[2].Ax = 0;
                T[2].Ay = -Math.Sin(tableDH[4] * DegToRad);
                T[2].Az = Math.Cos(tableDH[4] * DegToRad);
                T[2].Aw = 0;

                T[2].Px = tableDH[3];
                T[2].Py = -tableDH[8] * Math.Sin(tableDH[4] * DegToRad);
                T[2].Pz = tableDH[8] * Math.Cos(tableDH[4] * DegToRad);
                T[2].Pw = 1;
            } // T2

            {
                T[3].Nx = Math.Cos((jointValues[3] - 90) * DegToRad);
                T[3].Ny = Math.Cos(tableDH[7] * DegToRad) * Math.Sin((jointValues[3] - 90) * DegToRad);
                T[3].Nz = Math.Sin(tableDH[7] * DegToRad) * Math.Sin(jointValues[3] * DegToRad);
                T[3].Nw = 0;

                T[3].Ox = -Math.Sin((jointValues[3] - 90) * DegToRad);
                T[3].Oy = Math.Cos(tableDH[7] * DegToRad) * Math.Cos((jointValues[3] - 90) * DegToRad);
                T[3].Oz = Math.Sin(tableDH[7] * DegToRad) * Math.Cos((jointValues[3] - 90) * DegToRad);
                T[3].Ow = 0;

                T[3].Ax = 0;
                T[3].Ay = -Math.Sin(tableDH[7] * DegToRad);
                T[3].Az = Math.Cos(tableDH[7] * DegToRad);
                T[3].Aw = 0;

                T[3].Px = tableDH[6];
                T[3].Py = -tableDH[11] * Math.Sin(tableDH[7] * DegToRad);
                T[3].Pz = tableDH[11] * Math.Cos(tableDH[7] * DegToRad);
                T[3].Pw = 1;
            } // T3

            {
                T[4].Nx = Math.Cos((jointValues[4] + 180) * DegToRad);
                T[4].Ny = Math.Cos(tableDH[10] * DegToRad) * Math.Sin((jointValues[4] + 180) * DegToRad);
                T[4].Nz = Math.Sin(tableDH[10] * DegToRad) * Math.Sin((jointValues[4] + 180) * DegToRad);
                T[4].Nw = 0;

                T[4].Ox = -Math.Sin((jointValues[4] + 180) * DegToRad);
                T[4].Oy = Math.Cos(tableDH[10] * DegToRad) * Math.Cos((jointValues[4] + 180) * DegToRad);
                T[4].Oz = Math.Sin(tableDH[10] * DegToRad) * Math.Cos((jointValues[4] + 180) * DegToRad);
                T[4].Ow = 0;

                T[4].Ax = 0;
                T[4].Ay = -Math.Sin(tableDH[10] * DegToRad);
                T[4].Az = Math.Cos(tableDH[10] * DegToRad);
                T[4].Aw = 0;

                T[4].Px = tableDH[9];
                T[4].Py = -tableDH[14] * Math.Sin(tableDH[10] * DegToRad);
                T[4].Pz = tableDH[14] * Math.Cos(tableDH[10] * DegToRad);
                T[4].Pw = 1;
            } // T4

            {
                T[5].Nx = Math.Cos((-jointValues[5] + 180) * DegToRad);
                T[5].Ny = Math.Cos(tableDH[13] * DegToRad) * Math.Sin((-jointValues[5] + 180) * DegToRad);
                T[5].Nz = Math.Sin(tableDH[13] * DegToRad) * Math.Sin((-jointValues[5] + 180) * DegToRad);
                T[5].Nw = 0;

                T[5].Ox = -Math.Sin((-jointValues[5] + 180) * DegToRad);
                T[5].Oy = Math.Cos(tableDH[13] * DegToRad) * Math.Cos((-jointValues[5] + 180) * DegToRad);
                T[5].Oz = Math.Sin(tableDH[13] * DegToRad) * Math.Cos((-jointValues[5] + 180) * DegToRad);
                T[5].Ow = 0;

                T[5].Ax = 0;
                T[5].Ay = -Math.Sin(tableDH[13] * DegToRad);
                T[5].Az = Math.Cos(tableDH[13] * DegToRad);
                T[5].Aw = 0;

                T[5].Px = tableDH[12];
                T[5].Py = -tableDH[17] * Math.Sin(tableDH[13] * DegToRad);
                T[5].Pz = tableDH[17] * Math.Cos(tableDH[13] * DegToRad);
                T[5].Pw = 1;
            } // 5

            {
                T[6].Nx = Math.Cos((-jointValues[6] + 180) * DegToRad);
                T[6].Ny = Math.Cos(tableDH[16] * DegToRad) * Math.Sin((-jointValues[6] + 180) * DegToRad);
                T[6].Nz = Math.Sin(tableDH[16] * DegToRad) * Math.Sin((-jointValues[6] + 180) * DegToRad);
                T[6].Nw = 0;

                T[6].Ox = -Math.Sin((-jointValues[6] + 180) * DegToRad);
                T[6].Oy = Math.Cos(tableDH[16] * DegToRad) * Math.Cos((-jointValues[6] + 180) * DegToRad);
                T[6].Oz = Math.Sin(tableDH[16] * DegToRad) * Math.Cos((-jointValues[6] + 180) * DegToRad);
                T[6].Ow = 0;

                T[6].Ax = 0;
                T[6].Ay = -Math.Sin(tableDH[16] * DegToRad);
                T[6].Az = Math.Cos(tableDH[16] * DegToRad);
                T[6].Aw = 0;

                T[6].Px = tableDH[15];
                T[6].Py = -tableDH[20] * Math.Sin(tableDH[16] * DegToRad);
                T[6].Pz = tableDH[20] * Math.Cos(tableDH[16] * DegToRad);
                T[6].Pw = 1;
            } // 6

            //T[6].RotateAroundX(180);
            //T[6].RotateAroundY(180);

            //Matrix L0 = T[0];
            //Matrix L1 = T[0] * T[1];
            //Matrix L2 = T[0] * T[1] * T[2];
            //Matrix L3 = T[0] * T[1] * T[2] * T[3];
            //Matrix L4 = T[0] * T[1] * T[2] * T[3] * T[4];
            //Matrix L5 = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
            //Matrix L6 = T[0] * T[1] * T[2] * T[3] * T[4] * T[5] * T[6];

            Matrix tool0 = T[0];
            for (int i = 1; i < T.Length; i++)
            {
                //T[i].Uniform();
                tool0 *= T[i];
            }

            return tool0;
        }

        private Matrix extendedDHCalculations(double[] tableDH, double[] jointValues)
        {
            //Matrix T0 = new Matrix();
            Matrix[] T = new Matrix[7];

            { // Sin(0) = 0    ||  Cos(0) = 1
                T[0].Nx = Math.Cos(-jointValues[1] * DegToRad);
                T[0].Ny = Math.Sin(-jointValues[1] * DegToRad);
                T[0].Nz = 0;
                T[0].Nw = 0;

                T[0].Ox = Math.Sin(-jointValues[1] * DegToRad);
                T[0].Oy = -Math.Cos(-jointValues[1] * DegToRad);
                T[0].Oz = 0;
                T[0].Ow = 0;

                T[0].Ax = 0;
                T[0].Ay = 0;
                T[0].Az = -1;
                T[0].Aw = 0;

                T[0].Px = 0;
                T[0].Py = 0;
                T[0].Pz = 0;// tableDH[2];// <== Nemá mať efekt na guľu, mení Z v absolútnom smere - Vhodné na presun do Py, keďže to jediné nieje pohyblivé s aktuálnou DH tabuľkou
                T[0].Pw = 1;
            } // T0

            {
                T[1].Nx = 1;
                T[1].Ny = 0;
                T[1].Nz = 0;
                T[1].Nw = 0;

                T[1].Ox = 0;
                T[1].Oy = Math.Cos(tableDH[1] * DegToRad);
                T[1].Oz = Math.Sin(tableDH[1] * DegToRad);
                T[1].Ow = 0;

                T[1].Ax = 0;
                T[1].Ay = -Math.Sin(tableDH[1] * DegToRad);
                T[1].Az = Math.Cos(tableDH[1] * DegToRad);
                T[1].Aw = 0;

                T[1].Px = tableDH[0];
                T[1].Py = -tableDH[5] * Math.Sin(tableDH[1] * DegToRad);
                T[1].Pz = tableDH[5] * Math.Cos(tableDH[1] * DegToRad);
                T[1].Pw = 1;
            } // T1

            {
                T[2].Nx = Math.Cos((jointValues[2] + tableDH[24]) * DegToRad);
                T[2].Ny = Math.Cos(tableDH[4] * DegToRad) * Math.Sin((jointValues[2] + tableDH[24]) * DegToRad);
                T[2].Nz = Math.Sin(tableDH[4] * DegToRad) * Math.Sin((jointValues[2] + tableDH[24]) * DegToRad);
                T[2].Nw = 0;

                T[2].Ox = -Math.Sin((jointValues[2] + tableDH[24]) * DegToRad);
                T[2].Oy = Math.Cos(tableDH[4] * DegToRad) * Math.Cos((jointValues[2] + tableDH[24]) * DegToRad);
                T[2].Oz = Math.Sin(tableDH[4] * DegToRad) * Math.Cos((jointValues[2] + tableDH[24]) * DegToRad);
                T[2].Ow = 0;

                T[2].Ax = 0;
                T[2].Ay = -Math.Sin(tableDH[4] * DegToRad);
                T[2].Az = Math.Cos(tableDH[4] * DegToRad);
                T[2].Aw = 0;

                T[2].Px = tableDH[3];
                T[2].Py = -tableDH[8] * Math.Sin(tableDH[4] * DegToRad);
                T[2].Pz = tableDH[8] * Math.Cos(tableDH[4] * DegToRad);
                T[2].Pw = 1;
            } // T2

            {
                T[3].Nx = Math.Cos(((jointValues[3] + tableDH[25]) - 90) * DegToRad);
                T[3].Ny = Math.Cos(tableDH[7] * DegToRad) * Math.Sin(((jointValues[3] + tableDH[25]) - 90) * DegToRad);
                T[3].Nz = Math.Sin(tableDH[7] * DegToRad) * Math.Sin((jointValues[3] + tableDH[25]) * DegToRad);
                T[3].Nw = 0;

                T[3].Ox = -Math.Sin(((jointValues[3] + tableDH[25]) - 90) * DegToRad);
                T[3].Oy = Math.Cos(tableDH[7] * DegToRad) * Math.Cos(((jointValues[3] + tableDH[25]) - 90) * DegToRad);
                T[3].Oz = Math.Sin(tableDH[7] * DegToRad) * Math.Cos(((jointValues[3] + tableDH[25]) - 90) * DegToRad);
                T[3].Ow = 0;

                T[3].Ax = 0;
                T[3].Ay = -Math.Sin(tableDH[7] * DegToRad);
                T[3].Az = Math.Cos(tableDH[7] * DegToRad);
                T[3].Aw = 0;

                T[3].Px = tableDH[6];
                T[3].Py = -tableDH[11] * Math.Sin(tableDH[7] * DegToRad);
                T[3].Pz = tableDH[11] * Math.Cos(tableDH[7] * DegToRad);
                T[3].Pw = 1;
            } // T3

            {
                T[4].Nx = Math.Cos((jointValues[4] + tableDH[26] + 180) * DegToRad);
                T[4].Ny = Math.Cos(tableDH[10] * DegToRad) * Math.Sin((jointValues[4] + tableDH[26] + 180) * DegToRad);
                T[4].Nz = Math.Sin(tableDH[10] * DegToRad) * Math.Sin((jointValues[4] + tableDH[26] + 180) * DegToRad);
                T[4].Nw = 0;

                T[4].Ox = -Math.Sin((jointValues[4] + tableDH[26] + 180) * DegToRad);
                T[4].Oy = Math.Cos(tableDH[10] * DegToRad) * Math.Cos((jointValues[4] + tableDH[26] + 180) * DegToRad);
                T[4].Oz = Math.Sin(tableDH[10] * DegToRad) * Math.Cos((jointValues[4] + tableDH[26] + 180) * DegToRad);
                T[4].Ow = 0;

                T[4].Ax = 0;
                T[4].Ay = -Math.Sin(tableDH[10] * DegToRad);
                T[4].Az = Math.Cos(tableDH[10] * DegToRad);
                T[4].Aw = 0;

                T[4].Px = tableDH[9];
                T[4].Py = -tableDH[14] * Math.Sin(tableDH[10] * DegToRad);
                T[4].Pz = tableDH[14] * Math.Cos(tableDH[10] * DegToRad);
                T[4].Pw = 1;
            } // T4

            {
                T[5].Nx = Math.Cos((-jointValues[5] + tableDH[27] + 180) * DegToRad);
                T[5].Ny = Math.Cos(tableDH[13] * DegToRad) * Math.Sin((-jointValues[5] + tableDH[27] + 180) * DegToRad);
                T[5].Nz = Math.Sin(tableDH[13] * DegToRad) * Math.Sin((-jointValues[5] + tableDH[27] + 180) * DegToRad);
                T[5].Nw = 0;

                T[5].Ox = -Math.Sin((-jointValues[5] + tableDH[27] + 180) * DegToRad);
                T[5].Oy = Math.Cos(tableDH[13] * DegToRad) * Math.Cos((-jointValues[5] + tableDH[27] + 180) * DegToRad);
                T[5].Oz = Math.Sin(tableDH[13] * DegToRad) * Math.Cos((-jointValues[5] + tableDH[27] + 180) * DegToRad);
                T[5].Ow = 0;

                T[5].Ax = 0;
                T[5].Ay = -Math.Sin(tableDH[13] * DegToRad);
                T[5].Az = Math.Cos(tableDH[13] * DegToRad);
                T[5].Aw = 0;

                T[5].Px = tableDH[12];
                T[5].Py = -tableDH[17] * Math.Sin(tableDH[13] * DegToRad);
                T[5].Pz = tableDH[17] * Math.Cos(tableDH[13] * DegToRad);
                T[5].Pw = 1;
            } // T5

            {
                T[6].Nx = Math.Cos((-jointValues[6] + 180) * DegToRad);
                T[6].Ny = Math.Cos(tableDH[16] * DegToRad) * Math.Sin((-jointValues[6] + 180) * DegToRad);
                T[6].Nz = Math.Sin(tableDH[16] * DegToRad) * Math.Sin((-jointValues[6] + 180) * DegToRad);
                T[6].Nw = 0;

                T[6].Ox = -Math.Sin((-jointValues[6] + 180) * DegToRad);
                T[6].Oy = Math.Cos(tableDH[16] * DegToRad) * Math.Cos((-jointValues[6] + 180) * DegToRad);
                T[6].Oz = Math.Sin(tableDH[16] * DegToRad) * Math.Cos((-jointValues[6] + 180) * DegToRad);
                T[6].Ow = 0;

                T[6].Ax = 0;
                T[6].Ay = -Math.Sin(tableDH[16] * DegToRad);
                T[6].Az = Math.Cos(tableDH[16] * DegToRad);
                T[6].Aw = 0;

                T[6].Px = tableDH[15];
                T[6].Py = -tableDH[18] * Math.Sin(tableDH[16] * DegToRad);
                T[6].Pz = tableDH[18] * Math.Cos(tableDH[16] * DegToRad);
                T[6].Pw = 1;
            } // T6

            Matrix tool0 = new Matrix();
            tool0.SetP(new Vector3(tableDH[28], tableDH[29], tableDH[30]));
            tool0.SetWPR(new Vector3(tableDH[33], tableDH[32], tableDH[31]));

            //Matrix tool0 = T[0];
            for (int i = 0; i < T.Length; i++)
            {
                //T[i].Uniform();
                tool0 *= T[i];
            }

            return tool0;
        }

        void createGraph(double[] testTable)
        {
            DHPoints = new System.Windows.Media.PointCollection();
            System.Windows.Media.PointCollection points = new System.Windows.Media.PointCollection();
            graphOffset = (int)(DHPolyLine.Width / testTable.Length);
            for (int i = 0; i < testTable.Length; i++)
            {
                points.Add(new Point(i * graphOffset, 100));
            }
            DHPoints = points;
        }

        void refreshGraph(double[] startDHTable, double[] bestDH)
        {
            System.Windows.Media.PointCollection points = new System.Windows.Media.PointCollection();
            double[] deltaDouble = new double[bestDH.Length];
            
            for (int i = 0; i < bestDH.Length; i++)
            {
                var deltaVal = (startDHTable[i] - bestDH[i]);
                deltaDouble[i] = deltaVal;
                //points.Add(new Point(i * 25, deltaVal));
            }

            MinDHDelta = deltaDouble.Min();
            MaxDHDelta = deltaDouble.Max();

            double multipl = DHPolyLine.ActualHeight / deltaDouble.Select(x => Math.Abs(x)).Max() / 2.2;
            for (int i = 0; i < deltaDouble.Length; i++)
            {
                double newVal = -deltaDouble[i] * multipl + 100;
                points.Add(new Point(i * graphOffset, (int)newVal));
            }
            
            points.Freeze();
            DHPoints = points;
        }

        void MultiSphere_3DShow(List<CalibSphere> toEvalSpheres, double[] tableDH)
        {
            if (!isRenderReady)
            {
                return;
            }

            foreach (var sphere in toEvalSpheres)
            {

                SingleSphere_3DShow(generate3DPointCloud(sphere.robotPoses, sphere.scan_Lines, tableDH), toEvalSpheres.First() == sphere);
            }

            isRenderReady = false;
        }
        
        void SingleSphere_3DShow(Matrix[] Points, bool clearSpace = false)
        {
            var meshBuilder = new MeshBuilder(false, false);

            foreach (Matrix point in Points)
            {
                meshBuilder.AddSphere(new Point3D(point.Px / 100, point.Py / 100, point.Pz / 100), 0.003, 3, 2);
            }
            var mesh = meshBuilder.ToMesh();
            mesh.Freeze();

            Model.Dispatcher.InvokeAsync(() =>
            {
                if (clearSpace)
                {
                    Model.Children.Clear();
                }

                Model.Children.Add(new GeometryModel3D { Geometry = mesh, Material = MaterialHelper.CreateMaterial(System.Windows.Media.Colors.Green) });
            });

            PointCloud_Updated = true;
        }

        


        #endregion

        #region Files

        public void WritePLYbyTriggers(string uriPLY, double[][] robotPoses, Matrix[][] scan_Lines, double[] tableDH)
        {
            int j = 0;
            int n = 0;
            List<Matrix> subTrajectory = new List<Matrix>();
            List<Matrix[]> subScans = new List<Matrix[]>();
            ProgresActionName = "Writing files separated by trigger ...";
            for (int i = 0; i < robotPoses.Length - 1; i++)
            {
                subTrajectory.Add(extendedDHCalculations(tableDH, robotPoses[i]));
                subScans.Add(scan_Lines[i]);

                if (i < robotPoses.Length)
                {
                    Matrix nextPoint = extendedDHCalculations(tableDH, robotPoses[i + 1]);
                    var dist = distance(subTrajectory[j].Px, subTrajectory[j].Py, subTrajectory[j].Pz, nextPoint.Px, nextPoint.Py, nextPoint.Pz);

                    if (dist > 10)
                    {
                        string ext = uriPLY.Substring(uriPLY.Length - 4);
                        string newURI = uriPLY.Replace(ext, "-" + n + ext);
                        n++;

                        Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                        Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                        List<Matrix> pointCloud = new List<Matrix>();

                        for (int k = 0; k < subTrajectory.Count; k++)
                        {

                            pointCloud.AddRange(generate3DScanLine(subTrajectory[k], applyScannerMTX(Scanner_MTX, subScans[k])));
                        }

                        WriteOutputPLY(newURI, pointCloud.ToArray());
                        subTrajectory = new List<Matrix>();
                        subScans = new List<Matrix[]>();
                        j = -1;
                    }
                }
                j++;

                if (i > 0) { ProgressValue = (i / (float)robotPoses.Length) * 100; }
            }

            if (subTrajectory.Count > 0)
            {
                string ext = uriPLY.Substring(uriPLY.Length - 4);
                string newURI = uriPLY.Replace(ext, "-" + n + ext);
                n++;

                Scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                Scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                List<Matrix> pointCloud = new List<Matrix>();

                for (int k = 0; k < subTrajectory.Count; k++)
                {

                    pointCloud.AddRange(generate3DScanLine(subTrajectory[k], applyScannerMTX(Scanner_MTX, subScans[k])));
                }

                WriteOutputPLY(newURI, pointCloud.ToArray());
            }
        }

        void writeCalibReport(string uriTXT, double[][] robotPoses, Matrix[][] scan_Lines, string inputFileURI, double[] expectedPosition, double expectedRadius, double surfaceTolerance, double[] dh, bool writeDH = false)
        {
            if (File.Exists(uriTXT))
            {
                File.Delete(uriTXT);
            }

            var pointcloud = generate3DPointCloud(robotPoses, scan_Lines, dh);

            #region Evaluate Sphere parameters

            double[,] a = new double[pointcloud.Length, 4];
            double[] f = new double[pointcloud.Length];

            int i = 0;
            foreach (Matrix mtx in pointcloud)
            {
                a[i, 0] = mtx.Px * 2;
                a[i, 1] = mtx.Py * 2;
                a[i, 2] = mtx.Pz * 2;
                a[i, 3] = 1.0f;
                f[i] = (mtx.Px * mtx.Px) + (mtx.Py * mtx.Py) + (mtx.Pz * mtx.Pz);
                i++;
            }

            var aMatrix = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.DenseOfArray(a);
            var fVector = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(f);

            var cVector = MultipleRegression.QR(aMatrix, fVector);

            // solve for the radius
            double radius = System.Math.Sqrt((double)((cVector[0] * cVector[0]) + (cVector[1] * cVector[1]) + (cVector[2] * cVector[2]) + cVector[3]));

            double maxRadius = 0;
            double minRadius = double.MaxValue;
            double onSurfaceCount = 0;
            //double avgRadius = 0;

            foreach (Matrix point in pointcloud)
            {
                double dist = distance(cVector[0], cVector[1], cVector[2], (double)point.Px, (double)point.Py, (double)point.Pz);
                //avgRadius += dist;

                if (dist > maxRadius)
                {
                    maxRadius = dist;
                }

                if (dist < minRadius)
                {
                    minRadius = dist;
                }

                if (dist < (radius + surfaceTolerance) && (radius - surfaceTolerance) < dist)
                {
                    onSurfaceCount++;
                }
            }
            //avgRadius /= P.Length;
            double coverage = onSurfaceCount / pointcloud.Length;

            var radiusErr = Math.Abs(radius - expectedRadius);
            var minMax = Math.Abs(maxRadius - minRadius);
            //
            var error = minMax + radiusErr;
            error /= coverage;

            #endregion

            double[] avgPose = new double[robotPoses[0].Length];
            robotPoses[0].CopyTo(avgPose, 0);

            for (int n = 0; n < robotPoses.Length; n++)
            {
                for (int m = 0; m < avgPose.Length; m++)
                {
                    avgPose[m] += robotPoses[n][m];
                    avgPose[m] *= 0.5;
                }
            }

            string inlineDH = "";
            using (var reportFile = new StreamWriter(uriTXT, true, Encoding.UTF8))
            {
                reportFile.Write(inputFileURI + "\n\n");
                // Write temperatures
                //txtFile.WriteLine(string.Join(",", robotTemperatures.Select(x => x)));

                reportFile.Write("1. Max single line radius.......\t => " + expectedRadius + "\n");
                reportFile.Write("2. Sphere radius ...............\t => " + radius + "\n");
                reportFile.Write("3. Delta (1. <> 2.) ............\t => " + radiusErr + "\n");
                reportFile.Write("\n");
                reportFile.Write("4. MinMax ......................\t => " + minMax + "\n");
                reportFile.Write("5. Surface tolerance ...........\t => " + surfaceTolerance + "\n");
                reportFile.Write("6. Coverage in tolerance........\t => " + coverage * 100 + "%\n");
                reportFile.Write("7. Points in tolerance..........\t => " + onSurfaceCount + "\n");
                reportFile.Write("\n");
                reportFile.Write("8. Points used for calibration..\t => " + pointcloud.Length + "\n");
                reportFile.Write("9. Calibration error val .......\t => " + error + "\n");
                reportFile.Write("10.Finished in .................\t => " + FormatedTimeSTR + "\n");
                reportFile.Write("\n");
                reportFile.Write("");
                reportFile.Write(string.Format("11. Sphere position: ...........\t[{0:F6}, {1:F6}, {2:F6}]\n", cVector[0], cVector[1], cVector[2]));
                if (expectedPosition != null)
                {
                    reportFile.Write(string.Format("12. Distance from nominal: .....\t{0:F6}\n", distance(expectedPosition[0], expectedPosition[1], expectedPosition[2], cVector[0], cVector[1], cVector[2])));
                }

                reportFile.Write("\n");
                reportFile.Write("13. Result DH Parameters: \n");

                for (int j = 0; j < dh.Length; j++)
                {
                    reportFile.Write(string.Format("\t{0:F20}\n", dh[j]));

                    inlineDH += dh[j];
                    if (j < dh.Length-1)
                    {
                        inlineDH += ", ";
                    }
                }

                reportFile.Write("\n");

                //<add key="TableDH"  value = "350, 0, 0, 0, 90, -815, 850, 0, 0, 145, 90, 0, 0, -90, -820, 0, 90, 0, 0, 0, 170, 80, 0, 125, 0, 90, 0"/>
                reportFile.Write(string.Format("<add key=\"TableDH\"  value = \"{0:F20}\"/>", inlineDH));
            }

            if (writeDH)
            {
                string dhFileName = "DHZonesVals.dh";
                string dhFileURI = "";
                string[] uriParts = uriTXT.Split('\\');
                for (int u = 0; u < uriParts.Length - 1; u++) 
                {
                    dhFileURI += uriParts[u] + "\\";
                }
                dhFileURI += dhFileName;

                using (var dhFile = new StreamWriter(dhFileURI, true, Encoding.UTF8))
                {
                    dhFile.Write(string.Format("{0:F6}, {1:F6}, {2:F6} : {3:F6}, {4:F6}, {5:F6}, {6:F6}, {7:F6}, {8:F6} : {9}\n", 
                        cVector[0], cVector[1], cVector[2], avgPose[1], avgPose[2], avgPose[3], avgPose[4], avgPose[5], avgPose[6], inlineDH));
                }
            }

            if (Sounds)
            {
                SoundPlayer soundPlayer = new SoundPlayer(Properties.Resources.ExportedSound);
                soundPlayer.Play();
            }
        }

        async void write3DPointCloudTXT(string uriTXT, int[] robotTemperatures, double[][] robotPoses, Matrix[][] scan_Lines, double rejectOutlinersRatio, DHZones dhZones = null)
        {
            if (File.Exists(uriTXT))
            {
                File.Delete(uriTXT);
            }

            bool pathImprovements = false;
            if (rejectOutlinersRatio > 0)
            {
                pathImprovements = true;
            }
            List<Matrix> optimizedTrajectory = new List<Matrix>();
            List<Matrix> originalTrajectory = new List<Matrix>();
            List<double> timeStamps = new List<double>();
            Matrix[] optimizedSubPath = null;

            // For DH Zones only !!!
            Scanner_MTX.SetP(new Vector3(TableDH[19], TableDH[20], TableDH[21]));
            Scanner_MTX.SetWPR(new Vector3(TableDH[22], TableDH[23], TableDH[24]));
            Matrix fc = new Matrix();
             fc.SetP(new Vector3(0, 0, 275));
            Matrix focusPoint = Matrix.Multiply(Scanner_MTX, fc);

            Matrix[] arrScanner_MTXs = new Matrix[robotPoses.Length];
            // Optimize Path
            int j = 0;
            double[] tableDH = null;
            for (int i = 0; i < robotPoses.Length; i++)
            {
                if (dhZones != null)
                {
                    Matrix robotPos = extendedDHCalculations(TableDH, robotPoses[i]);
                    robotPos.TranslateRelative(focusPoint.GetP());
                    tableDH = dhZones.GetDHTable(robotPos.GetP(), robotPoses[i]);
                }
                arrScanner_MTXs[i].SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                arrScanner_MTXs[i].SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                originalTrajectory.Add(extendedDHCalculations(tableDH, robotPoses[i]));
                timeStamps.Add(robotPoses[i][0]);

                if (i < robotPoses.Length - 1 && pathImprovements)
                {
                    Matrix nextPoint = extendedDHCalculations(tableDH, robotPoses[i + 1]);
                    var dist = distance(originalTrajectory[j].Px, originalTrajectory[j].Py, originalTrajectory[j].Pz, nextPoint.Px, nextPoint.Py, nextPoint.Pz);

                    if (dist > 10)
                    {
                        optimizedSubPath = pathOptimiser(originalTrajectory.ToArray(), timeStamps.ToArray(), rejectOutlinersRatio, 0); 
                        optimizedTrajectory.AddRange(optimizedSubPath);
                        originalTrajectory = new List<Matrix>();
                        timeStamps = new List<double>();
                        j = -1;
                    }
                }
                j++;

                if (i > 0) { ProgressValue = (i / (float)robotPoses.Length) * 100; }
            }
            if (pathImprovements)
            {
                optimizedSubPath = pathOptimiser(originalTrajectory.ToArray(), timeStamps.ToArray(), rejectOutlinersRatio, 0); // 10 polôh začiatku a konca odseknúť - vlny
                optimizedTrajectory.AddRange(optimizedSubPath);


                //for (int i = 0; i < scan_Lines.Length; i++)
                //{
                //    scan_Lines[i] = clearScanlinesInPolynom(scan_Lines[i], ScanlineMinMaxClear);
                //}
            }
            else
            {
                optimizedTrajectory = originalTrajectory;
            }

            ProgressValue = 0;


            //Process myProcess = Process.GetCurrentProcess();
            //long usedMemory = myProcess.PrivateMemorySize64;

            if (ActiveProcesses.Count >= 3)
            {
                Task.WaitAll(ActiveProcesses.ToArray());
                ActiveProcesses = new List<Task>();
            }

            bool scanImprovements = false;
            ActiveProcesses.Add(Task.Factory.StartNew(() =>
            {
                //Matrix scanner_MTX = new Matrix();

                if (scanImprovements)
                {
                    scan_Lines = cleanAllScanlinesInPoly(scan_Lines, DirectRegressionMethod.QR);
                }

                string uriBound = uriTXT.Substring(0, uriTXT.Length - 4) + "_b.txt";

                using (var txtFile = new StreamWriter(uriTXT, true, Encoding.UTF8))
                {
                    // Write temperatures
                    txtFile.WriteLine(string.Join(",", robotTemperatures.Select(x => x)));

                    using (var boundFile = new StreamWriter(uriBound, true, Encoding.UTF8))
                    {
                        //Matrix previus_MTX = optimizedTrajectory[0];

                        for (int i = 0; i < scan_Lines.Length; i++)
                        {
                            Matrix robot_MTX = optimizedTrajectory[i];

                            if (robot_MTX.Px < 0.01 && !(robot_MTX.Px < -0.1))
                            {
                                continue;
                            }

                            //scanner_MTX.SetP(new Vector3(tableDH[19], tableDH[20], tableDH[21]));
                            //scanner_MTX.SetWPR(new Vector3(tableDH[22], tableDH[23], tableDH[24]));

                            Matrix[] scanner_Points = applyScannerMTX(arrScanner_MTXs[i], scan_Lines[i]);


                            Matrix[] newArr = generate3DScanLine(robot_MTX, scanner_Points);

                            Matrix[] newArrBound = extractBoundary(newArr);

                            txtFile.WriteLine(string.Join("\n", newArr.Select(x => $"{Math.Round(x.Px, 4)},{Math.Round(x.Py, 4)},{Math.Round(x.Pz, 4)}")));
                            boundFile.WriteLine(string.Join("\n", newArrBound.Select(x => $"{Math.Round(x.Px, 4)},{Math.Round(x.Py, 4)},{Math.Round(x.Pz, 4)}")));

                            ProgresActionName = "Pointcloud generation and writing ...";
                            if (i > 0) { ProgressValue = (i / (float)robotPoses.Length) * 100; }
                        }
                    }
                }
                ProgresActionName = "Finished ...";
                ProgressValue = 0;
            }));
        }

        (int[] robotTemperatures, double[][] robotPoses, Matrix[][] scanLines) readPLCData(string uri)
        {
            bool xzpFileExist = true;
            if (!System.IO.File.Exists(uri + ".rpc"))
            {
                ProgresActionName = "Súbor neexistuje: " + uri + ".rpc";
                return (null, null, null);
            }
            if (!System.IO.File.Exists(uri + ".xzpc"))
            {
                //ProgresActionName = "Súbor neexistuje: " + uri + ".xzpc";
                //return (null, null, null);
                xzpFileExist = false;
                if (!System.IO.File.Exists(uri + ".bin"))
                {
                    ProgresActionName = "Súbor neexistuje" + uri + ".xzpc/.bin";
                    return (null, null, null);
                }
            }

            //List<double> allZvals = new List<double>();

            string[] linesRobot = System.IO.File.ReadAllLines(uri + ".rpc");
            string[] linesXZ = null;
            Matrix[][] testXZ = null;

            var testRobot = readRPCfile(linesRobot);

            if (xzpFileExist)
            {
                linesXZ = System.IO.File.ReadAllLines(uri + ".xzpc");
                testXZ = readXZfile(linesXZ);
            }
            else
            {
                testXZ = readBINfile(uri + ".bin");
            }

            if (testRobot.robotPoses.Length != testXZ.Length)
            {
                ProgresActionName = "Nezhoda riadkov: " + uri;
                return (null, null, null);
            }

            return (testRobot.robotTemperatures, testRobot.robotPoses, testXZ);
        }

        (int[] robotTemperatures, double[][] robotPoses) readRPCfile(string[] linesRobot)
        {
            // Get robot temperatures
            string[] splitTempLine = linesRobot[0].Split(';');
            int[] robotTemperatures = new int[6];
            for (int i = 0; i < 6; i++)
            {
                robotTemperatures[i] = int.Parse(splitTempLine[i]);
            }

            List<double[]> robotList = new List<double[]>();

            bool readingFailed = false;
            for (int i = 1; i < linesRobot.Length; i++)
            {
                //if (i > 1500) { break; }


                // Prepare Robot line
                string lineRobot = linesRobot[i];
                string[] splitRobotLine = lineRobot.Split(';');

                // Get Robot data
                double[] joints = new double[splitRobotLine.Length];
                for (int j = 0; j < joints.Length - 1; j++)
                {
                    joints[j] = double.Parse(splitRobotLine[j]);
                }
                robotList.Add(joints);
            }

            //allZvals.Sort();

            if (readingFailed)
            {
                return (null, null);
            }

            return (robotTemperatures, robotList.ToArray());
        }

        Matrix[][] readXZfile(string[] linesXZ)
        {
            List<Matrix[]> scanList = new List<Matrix[]>();

            List<Matrix> scanLine = new List<Matrix>();
            bool readingFailed = false;
            for (int i = 0; i < linesXZ.Length; i++)
            {
                // Prepare XZ line
                string lineXZ = linesXZ[i];
                string[] splitXZLine = lineXZ.Split(';');

                // Get Scan data
                int index = 0;
                for (int j = 0; j < splitXZLine.Length; j += 2)
                {
                    try
                    {
                        double x = 0;
                        double y = double.Parse(splitXZLine[j]);
                        double z = double.Parse(splitXZLine[j + 1]);

                        if (z == 0 || z == 59.464 || z == 106.16)// || z > 260 || z < 240)
                        {
                            continue;
                        }

                        //allZvals.Add(z);

                        Matrix pointMTX = Matrix.Zero;
                        pointMTX.SetP(new Vector3(x, y, z));
                        scanLine.Add(pointMTX);
                        index++;

                    }
                    catch
                    {
                        print(string.Format("Reading failed at line {0}", j));
                    }
                }

                if (ImportTCP0) { scanLine.Add(Matrix.Zero); } // <<<<==== Add Robot TCP positions (point 0, 0, 0)

                scanList.Add(scanLine.ToArray());

                scanLine.Clear();

                //ProgresActionName = "Reading files: " + uri;
                if (i > 0) { ProgressValue = (i / (float)linesXZ.Length) * 100; }
            }

            //allZvals.Sort();

            if (readingFailed)
            {
                return (null);
            }

            return scanList.ToArray();
        }

        Matrix[][] readBINfile(string pathSource)
        {
            byte[] bytes;
            using (FileStream fsSource = new FileStream(pathSource,
                        FileMode.Open, FileAccess.Read))
            {

                // Read the source file into a byte array.
                bytes = new byte[fsSource.Length];
                int numBytesToRead = (int)fsSource.Length;
                int numBytesRead = 0;
                while (numBytesToRead > 0)
                {
                    // Read may return anything from 0 to numBytesToRead.
                    int n = fsSource.Read(bytes, numBytesRead, numBytesToRead);

                    // Break when the end of the file is reached.
                    if (n == 0)
                        break;

                    numBytesRead += n;
                    numBytesToRead -= n;
                }
            }

            int lineLength = 4096 * 4 + 2;
            int numbLength = 4;
            int lineCount = bytes.Length / lineLength;
            int numStep = numbLength * 2;

            List<Matrix[]> scanList = new List<Matrix[]>();
            List<Matrix> scanLine = new List<Matrix>();
            Matrix pointMTX = new Matrix();
            int countZeroZ = 0;

            List<Single> listVals = new List<Single>();
            for (int i = 0; i < bytes.Length; i += lineLength)
            {
                scanLine = new List<Matrix>();
                for (int j = 0; j < lineLength; j += numStep)
                {
                    if (bytes.Length <= i + j + 8 || lineLength < j + 8)
                    {
                        break;
                    }

                    byte[] valXbytes = new byte[numbLength];
                    byte[] valZbytes = new byte[numbLength];

                    for (int k = 0; k < numbLength; k++)
                    {
                        valXbytes[k] = bytes[i + j + k];
                        valZbytes[k] = bytes[i + j + k + 4];
                    }

                    Array.Reverse(valXbytes);
                    Array.Reverse(valZbytes);

                    pointMTX.Py = BitConverter.ToSingle(valXbytes, 0);
                    pointMTX.Pz = BitConverter.ToSingle(valZbytes, 0);

                    if (pointMTX.Pz < 110)
                    {
                        countZeroZ++;
                        continue;
                    }

                    //if (Math.Round(pointMTX.Pz, 2) == 59.45f || Math.Round(pointMTX.Pz, 2) == 106.16)
                    //{
                    //    countZeroZ++;
                    //    continue;
                    //}
                    scanLine.Add(pointMTX);
                }
                scanList.Add(scanLine.ToArray());
            }
            return scanList.ToArray();
        }


        public void WriteOutputPLY(string uriPLY, Matrix[] pointCloud)
        {
            if (File.Exists(uriPLY))
            {
                File.Delete(uriPLY);
            }

            string strHeader = "ply\n";
            strHeader += "format ascii 1.0\n";
            strHeader += string.Format("element vertex {0}\n", pointCloud.Length);
            strHeader += "property double x\n";
            strHeader += "property double y\n";
            strHeader += "property double z\n";
            strHeader += "element face 0\n";
            strHeader += "property list uchar int vertex_indices\n";
            strHeader += "end_header\n";

            File.AppendAllText(uriPLY, strHeader);
            File.AppendAllText(uriPLY, string.Join("\n", pointCloud.Select(x => $"{x.Px}\t {x.Py}\t {x.Pz}")));

            //Console.Beep();
        }


        #endregion

        #region Tools

        [DllImport("Powrprof.dll", CharSet = CharSet.Auto, ExactSpelling = true)]
        public static extern bool SetSuspendState(bool hiberate, bool forceCritical, bool disableWakeEvent);


        int printCount = 0;
        void print(string str)
        {
            Console.WriteLine(printCount + ": " + str);
            printCount++;
        }

        Matrix[][] cleanAllScanlinesInPoly(Matrix[][] scan_Lines, DirectRegressionMethod regressMethod = DirectRegressionMethod.QR)
        {
            List<Matrix[]> newScan_Lines = new List<Matrix[]>();

            int active = 0;
            for (int i = 0; i < scan_Lines.Length; i++)
            {
                newScan_Lines.Add(cleanScanlinesInPolynom2(scan_Lines[i], 1, regressMethod));

                ProgresActionName = "Optimising scanlines ...";
                if (i > 0) { ProgressValue = (i / (float)scan_Lines.Length) * 100; }

                active++;
            }

            return newScan_Lines.ToArray();
        }

        Matrix[] cleanScanlinesInPolynom2(Matrix[] sharpCloud, float cutOffFromPolyDist, DirectRegressionMethod regressMethod = DirectRegressionMethod.QR)
        {
            if (sharpCloud.Length < 100) { return new Matrix[0]; }

            #region Polynom data preparation

            List<double> yVals = new List<double>();
            List<double> zVals = new List<double>();

            List<List<double>> all_YVals = new List<List<double>>();
            List<List<double>> all_ZVals = new List<List<double>>();

            double lastY = sharpCloud[0].Py;
            double lastZ = sharpCloud[0].Pz;

            for (int index = 0; index < sharpCloud.Length - 1; index++)
            {
                double dist = Math.Sqrt(((sharpCloud[index].Py - lastY) * (sharpCloud[index].Py - lastY)) + ((sharpCloud[index].Pz - lastZ) * (sharpCloud[index].Pz - lastZ)));

                if (dist > cutOffFromPolyDist)
                {
                    all_YVals.Add(yVals);
                    all_ZVals.Add(zVals);

                    yVals = new List<double>();
                    zVals = new List<double>();
                    //continue;
                }

                yVals.Add(sharpCloud[index].Py);
                zVals.Add(sharpCloud[index].Pz);

                lastY = sharpCloud[index].Py;
                lastZ = sharpCloud[index].Pz;

            }
            if (yVals.Count > 10)
            {
                all_YVals.Add(yVals);
                all_ZVals.Add(zVals);

                yVals = null;
                zVals = null;
            }

            #endregion Polynom data preparation


            #region Filterout data - compare with polynomial curve

            List<Matrix> cleanCloudCurver = new List<Matrix>();

            for (int n = 0; n < all_YVals.Count; n++)
            {
                int pointCount = all_YVals[n].Count - 1;
                if (pointCount < 10)
                {
                    continue;
                }

                // Polynomial fit
                var resultFuncZ = MathNet.Numerics.Polynomial.Fit(all_YVals[n].ToArray(), all_ZVals[n].ToArray(), all_YVals[n].Count / 50, regressMethod);



                for (int i = 0; i < pointCount; i++)
                {


                    double newZ = MathNet.Numerics.Polynomial.Evaluate(all_YVals[n][i], resultFuncZ.Coefficients);


                    Matrix newMTX = new Matrix();

                    newMTX.Py = (float)all_YVals[n][i];
                    newMTX.Pz = (float)newZ;
                    cleanCloudCurver.Add(newMTX);
                }

            }

            #endregion Filterout data - compare with polynomial curve


            return cleanCloudCurver.ToArray();
        }

        Matrix[] cleanScanlinesInPolynom(Matrix[] sharpCloud, double scanlineMinMaxClear, int polyOrder = 2, DirectRegressionMethod regressMethod = DirectRegressionMethod.NormalEquations)
        {
            if (sharpCloud.Length < 100)
            {
                return new Matrix[0];
            }

            double[] yVals = new double[sharpCloud.Length-1];
            double[] zVals = new double[sharpCloud.Length-1];

            for (int index = 0; index < sharpCloud.Length-1; index++)
            {
                yVals[index] = sharpCloud[index].Py;
                zVals[index] = sharpCloud[index].Pz;
            }


            var resultFuncZ = MathNet.Numerics.Polynomial.Fit(yVals, zVals, polyOrder, regressMethod);


            int pointCount = sharpCloud.Length - 1;
            int cutEnds = (int)(pointCount * 0.1);

            //double[] newZs = new double[pointCount];
            //Matrix[] cleanCloudCurver = new Matrix[pointCount];
            List<Matrix> cleanCloudCurver = new List<Matrix>();
            for (int i = 0; i < pointCount; i++)
            {
                //Matrix newMTX = new Matrix();

                double y = sharpCloud[i].Py;

                double newZ = MathNet.Numerics.Polynomial.Evaluate(y, resultFuncZ.Coefficients);

                double diff = Math.Abs(newZ - sharpCloud[i].Pz);

                if (diff <= scanlineMinMaxClear)
                {
                    cleanCloudCurver.Add(sharpCloud[i]);
                }

                //newMTX.SetP(new Vector3(0, y, newZ));
                //
                //cleanCloudCurver[i] = newMTX;
            }

            //var subArray = cleanCloudCurver.Skip(cutEnds).Take(cleanCloudCurver.Length-2*cutEnds);

            var ratio = (double)cleanCloudCurver.Count / sharpCloud.Length * 100;

            return cleanCloudCurver.ToArray();
        }

        Matrix[] clearScanlinesInCircle(Matrix[] sharpCloud, double surfaceTolerance = 0.1, double minRadius = 20)
        {
            int numPoints = sharpCloud.Length;
            if (numPoints < 10)
            {
                return new Matrix[0];
            }

            double[,] a = new double[numPoints, 3];
            double[] f = new double[numPoints];

            int i = 0;
            foreach (Matrix mtx in sharpCloud)
            {
                a[i, 0] = mtx.Py * 2;
                a[i, 1] = mtx.Pz * 2;
                a[i, 2] = 1.0f;
                f[i] = (mtx.Py * mtx.Py) + (mtx.Pz * mtx.Pz);
                i++;
            }

            var aMatrix = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.DenseOfArray(a);
            var fVector = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(f);

            var cVector = MultipleRegression.QR(aMatrix, fVector);

            // solve for the radius
            //double t = (double)((cVector[0] * cVector[0]) + (cVector[1] * cVector[1]) + cVector[2]);
            double radius = Math.Sqrt((double)((cVector[0] * cVector[0]) + (cVector[1] * cVector[1]) + cVector[2]));

            // filter out small radius -- too close to poles
            if (radius < minRadius)
            {
                return new Matrix[0];
            }

            // ExpectedSphereRadius
            if (GetExpectedRadiusFromLines)
            {
                if (radius > ExpectedSphereRadius)
                {
                    ExpectedSphereRadius = radius;
                }
            }

            List<Matrix> clearScanLine = new List<Matrix>();

            foreach (Matrix point in sharpCloud)
            {
                double dist = distance(0, cVector[0], cVector[1], (double)point.Px, (double)point.Py, (double)point.Pz);

                if (dist < (radius + surfaceTolerance) && (radius - surfaceTolerance) < dist)
                {
                    clearScanLine.Add(point);
                }
            }

            var ratio = (double)clearScanLine.Count / sharpCloud.Length * 100;

            return clearScanLine.ToArray();
        }

        Matrix[] simplePointCountReduction(Matrix[] sharpCloud, int reqPointCount = 100)
        {
            int incr = 0;
            if (reqPointCount <= 2)
            {
                incr = sharpCloud.Length - 1;
            }
            else
            {
                incr = (int)(sharpCloud.Length / reqPointCount);
                if (sharpCloud.Length <= reqPointCount || incr == 1)
                {
                    return sharpCloud;
                }
            }

            List<Matrix> reducedCloud = new List<Matrix>();

            for (int i = 0; i < sharpCloud.Length; i += incr)
            {
                reducedCloud.Add(sharpCloud[i]);
            }

            return reducedCloud.ToArray();
        }

        Matrix[][] scanlineSubArray(Matrix[][] origScanlines, double start, double end)
        {
            if ((start > end) || (end > 1) || (end < 0))
            {
                return origScanlines;
            }

            Matrix[][] newScanlines = new Matrix[origScanlines.Length][];

            for (int i = 0; i < origScanlines.Length; i++)
            {
                int startIndex = (int)(origScanlines[i].Length * start);
                int endIndex = (int)(origScanlines[i].Length * end);

                List<Matrix> sLine = new List<Matrix>();
                for (int j = startIndex; j < endIndex; j++)
                {
                    sLine.Add(origScanlines[i][j]);
                }
                newScanlines[i] = sLine.ToArray();
            }

            return newScanlines;
        }

        Matrix[] scanlineSmooth(Matrix[] sharpCloud, bool withNoise = false, double noiseAmp = 0.2)
        {
            if (sharpCloud.Length < (SL_OrderDivisor * 10))
            {
                return sharpCloud;
            }

            int valueQR = (int)(sharpCloud.Length / SL_OrderDivisor);

            //if (valueQR > 25) { valueQR = 25; }

            //double[] xVals = new double[sharpCloud.Length];
            double[] yVals = new double[sharpCloud.Length];
            double[] zVals = new double[sharpCloud.Length];

            for (int index = 0; index < sharpCloud.Length; index++)
            {
                //xVals[index] = sharpCloud[index].Px;
                yVals[index] = sharpCloud[index].Py;
                zVals[index] = sharpCloud[index].Pz;
            }


            //var resultFuncZ = MathNet.Numerics.Polynomial.Fit(yVals, zVals, valueQR, DirectRegressionMethod.QR);
            var resultFuncZ = MathNet.Numerics.Polynomial.Fit(yVals, zVals, valueQR, DirectRegressionMethod.QR);

            int pointCount = sharpCloud.Length / SmoothPointCountDiv; //SmoothPointCount;
            int cutOff = (int)(pointCount * 0.25);
            if (cutOff < 5) { cutOff = 5; }
            double startPy = sharpCloud[cutOff].Py;
            double endPy = sharpCloud[sharpCloud.Length - cutOff].Py;


            Matrix[] cleanCloudCurver = new Matrix[pointCount];
            for (int i = 0; i < pointCount; i++)
            //foreach (Matrix point in originalCloud)
            {
                Matrix newMTX = new Matrix();
                double y = remapValue(i, 0, pointCount, (double)startPy, (double)endPy);

                double newX = 0;
                //double newX = MathNet.Numerics.Polynomial.Evaluate(y, resultFuncX.Coefficients);
                double newZ = MathNet.Numerics.Polynomial.Evaluate(y, resultFuncZ.Coefficients);

                if (withNoise)
                {
                    Random rnd = new Random();
                    double rndX = (rnd.NextDouble() - 0.5) * noiseAmp * 50;
                    double rndY = (rnd.NextDouble() - 0.5) * noiseAmp * 50;
                    double rndZ = (rnd.NextDouble() - 0.5) * noiseAmp * 50;
                    newMTX.SetP(new Vector3(newX + rndX, y + rndY, newZ + rndZ));
                }
                else
                {
                    newMTX.SetP(new Vector3(newX, y, newZ));
                }

                //double res = MathNet.Numerics.Polynomial.Evaluate(point.Py, resultFunc.Coefficients);
                //newMTX.SetP(new Vector3(0, point.Py, res));

                //cleanCloudCurver.Add(newMTX);
                cleanCloudCurver[i] = newMTX;
            }

            return cleanCloudCurver;
        }

        double distance(double x1, double y1, double z1, double x2, double y2, double z2)
        {
            double a = (x2 - x1);
            double b = (y2 - y1);
            double c = (z2 - z1);

            return (double)Math.Sqrt((a * a) + (b * b) + (c * c));
        }

        double remapValue(double value, double low1, double high1, double low2, double high2)
        {
            if (value <= low1) { return low2; }
            if (value >= high1) { return high2; }
            return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
        }

        async void StopWatch(DateTime startTime)
        {
            while (InProgress)
            {
                FormatedTimeSTR = (DateTime.Now - startTime).ToString();
                await Task.Delay(100);
            }
        }

        #endregion

        #region Buttons
        private void btn_PLCdata_2_PointCloud(object sender, RoutedEventArgs e)
        {
            Buttons_Enabled = false;
            InProgress = true;

            //Task.Run(() => plcData_2_PointCloud_TCPIP());
            Task.Run(() => plcData_2_PointCloud_File());
            Task.Run(() => StopWatch(DateTime.Now));
        }

        private void btn_SingleSphereCalibrations(object sender, RoutedEventArgs e)
        {
            Buttons_Enabled = false;
            if (!ReadConfigValues())
            {
                ProgresActionName = "Error in config file syntax";
                return;
            }

            createGraph(TableDH);
            InProgress = true;


            Task.Run(() => calibrationOnSphere());
            Task.Run(() => StopWatch(DateTime.Now));
        }

        private void btn_MultiSphereCalibrations(object sender, RoutedEventArgs e)
        {
            Buttons_Enabled = false;
            if (!ReadConfigValues())
            {
                ProgresActionName = "Error in config file syntax";
                return;
            }

            createGraph(TableDH);

            InProgress = true;
            Task.Run(() => calibrationOnMultiSphere());
            Task.Run(() => StopWatch(DateTime.Now));
        }

        private void btn_STOP(object sender, RoutedEventArgs e)
        {

            //FFT_Testy();
            //return;

            if (InProgress)
            {
                StopError = double.MaxValue;
                IsEffective = false;
                InProgress = false;
            }
            else
            {
                System.Environment.Exit(1);
            }
        }
        #endregion

        #region Properties with Event

        private Model3DGroup model;

        public Model3DGroup Model
        {
            get { return model; }
            set
            {
                model = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Model3DGroup"));
            }
        }

        private int outFileCounter;
        public int OutFileCounter
        {
            get { return outFileCounter; }
            set
            {
                outFileCounter = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("OutFileCounter"));
            }
        }

        private string progresActionName;
        public string ProgresActionName
        {
            get { return progresActionName; }
            set
            {
                progresActionName = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("ProgresActionName"));
            }
        }

        private double progressValue;
        public double ProgressValue
        {
            get { return progressValue; }
            set
            {
                progressValue = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("ProgressValue"));
            }
        }

        private double currentError;
        public double CurrentError
        {
            get { return currentError; }
            set
            {
                currentError = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("CurrentError"));
            }
        }

        private double bestError;
        public double BestError
        {
            get { return bestError; }
            set
            {
                bestError = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("BestError"));
            }
        }

        private double currentIncrement;
        public double CurrentIncrement
        {
            get { return currentIncrement; }
            set
            {
                currentIncrement = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("CurrentIncrement"));
            }
        }

        private bool buttons_Enabled;
        public bool Buttons_Enabled
        {
            get { return buttons_Enabled; }
            set
            {
                buttons_Enabled = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Buttons_Enabled"));
            }
        }

        private string formatedTimeSTR;
        public string FormatedTimeSTR
        {
            get { return formatedTimeSTR; }
            set
            {
                formatedTimeSTR = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("FormatedTimeSTR"));
            }
        }

        private double maxDHDelta;
        public double MaxDHDelta
        {
            get { return maxDHDelta; }
            set 
            { 
                maxDHDelta = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("MaxDHDelta"));
            }
        }

        private double minDHDelta;
        public double MinDHDelta
        {
            get { return minDHDelta; }
            set
            {
                minDHDelta = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("MinDHDelta"));
            }
        }


        private System.Windows.Media.PointCollection dhPoints;

        public System.Windows.Media.PointCollection DHPoints
        {
            get { return dhPoints; }
            set { 
                dhPoints = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("DHPoints"));
            }
        }


        #endregion

        public event PropertyChangedEventHandler PropertyChanged;
    }

    class CalibSphere
    {
        public double[][] robotPoses = null;
        public Matrix[][] scan_Lines = null;
        public double[] expectedPosition = null;
        public double expectedRadius = -1;
        public double surfaceTolerance = -1;
    }

    class DHZones
    {
        public double[][] ZonesDHtables;
        public double[][] ZoneJointVals;
        public double[][] ZonePositions;
        public double MinHalfDistance = 0;

        public DHZones(double[][] allTablesDH) 
        {
            if(allTablesDH.Length == 0)
            {
                return;
            }
            ZonesDHtables = allTablesDH;
        }

        public DHZones(string dhFileURI)
        {

            if (!System.IO.File.Exists(dhFileURI))
            {
                return;
            }

            string[] linesDH = System.IO.File.ReadAllLines(dhFileURI);

            ZonesDHtables = new double[linesDH.Length][];
            ZoneJointVals = new double[linesDH.Length][];
            ZonePositions = new double[linesDH.Length][];

            for (int i = 0; i < linesDH.Length; i++)
            {
                string[] separTypes = linesDH[i].Split(':');

                string[] splitPosition = separTypes[0].Split(",");
                string[] splitJoint = separTypes[1].Split(",");
                string[] splitDHs = separTypes[2].Split(",");

                ZonePositions[i] = new double[splitPosition.Length];
                ZoneJointVals[i] = new double[splitJoint.Length];
                ZonesDHtables[i] = new double[splitDHs.Length];

                for (int j = 0; j < splitPosition.Length; j++)
                {
                    ZonePositions[i][j] = double.Parse(splitPosition[j]);
                }

                for (int j = 0; j < splitJoint.Length; j++)
                {
                    ZoneJointVals[i][j] = double.Parse(splitJoint[j]);
                }

                for (int j = 0; j < splitDHs.Length; j++)
                {
                    ZonesDHtables[i][j] = double.Parse(splitDHs[j]);
                }

            }

            double minDistance = double.MaxValue;
            for (int i = 0; i < ZonePositions.Length; i++)
            {
                for (int j = 0; j < ZonePositions.Length; j++)
                {
                    double dist = distance(ZonePositions[i][0], ZonePositions[i][1], ZonePositions[i][2], ZonePositions[j][0], ZonePositions[j][1], ZonePositions[j][2]);
                    if (dist == 0) { continue; }
                    if (dist < minDistance) { minDistance = dist; }
                }
            }

            MinHalfDistance = minDistance * 0.5;
        }

        public double[] GetDHTable(Vector3 tcpVec, double[] joints)
        {
            return GetDHTable(new double[] { tcpVec.X, tcpVec.Y, tcpVec.Z }, joints);
        }

        //public double[] GetDHTable(double[] tcpPosition)
        //{
        //    double[] resultDH = new double[ZonesDHtables[0].Length];
        //    double[] weights = new double[ZonesDHtables.Length];
        //    double[] distances = new double[ZonesDHtables.Length];
        //
        //    // Get distances fro current position to all calibration center zones
        //    for (int i = 0; i < ZonesDHtables.Length; i++)
        //    {
        //        distances[i] = distance(ZonePositions[i][0], ZonePositions[i][1], ZonePositions[i][2], tcpPosition[0], tcpPosition[1], tcpPosition[2]);
        //    }
        //    double sum = distances.Sum();
        //
        //    // Calculate weights of the calibration zones
        //    for (int i = 0; i < distances.Length; i++)
        //  {
        //      weights[i] = ((sum - distances[i]) / sum);
        //  }
        //    sum = weights.Sum();
        //
        //    for (int i = 0; i < distances.Length; i++)
        //  {
        //      weights[i] /= sum;
        //  }
        //
        //    // Generate new DH table by combining af all DH tables according their weights in current position
        //    for (int i = 0; i < resultDH.Length; i++)
        //  {
        //      double newVal = 0;
        //      for (int j = 0; j < weights.Length; j++)
        //      {
        //          newVal += ZonesDHtables[j][i] * weights[j];
        //      }
        //      resultDH[i] = newVal;
        //  }
        //
        //    return resultDH;
        //}

        public double[] GetDHTable(double[] tcpPosition, double[] joints) ////////////////// Musím vyhodnocivať             Vzdialenosť + Podobnosť ....
        {
            double[] resultDH = new double[ZonesDHtables[0].Length];
            double[] weights = new double[ZonesDHtables.Length];
            double[] distances = new double[ZonesDHtables.Length];

            // Get distances fro current position to all calibration center zones
            for (int i = 0; i < ZonesDHtables.Length; i++)
            {
                double dist = distance(ZonePositions[i][0], ZonePositions[i][1], ZonePositions[i][2], tcpPosition[0], tcpPosition[1], tcpPosition[2]);
                if (dist > MinHalfDistance) 
                {
                    distances[i] = -1;
                    continue;
                }
                distances[i] = dist;
                distances[i] += getjointDistance(ZoneJointVals[i], joints);
            }
            double sum = distances.Sum();
            if (sum <= 0)
            {
                for (int i = 0; i < ZonesDHtables.Length; i++)
                {
                    double dist = distance(ZonePositions[i][0], ZonePositions[i][1], ZonePositions[i][2], tcpPosition[0], tcpPosition[1], tcpPosition[2]);
                    distances[i] = dist;
                    distances[i] += getjointDistance(ZoneJointVals[i], joints);
                }
            }

            // Calculate weights of the calibration zones
            for (int i = 0; i < distances.Length; i++)
            {
                if (distances[i] < 0)
                {
                    weights[i] = 0;
                    continue;
                }
                weights[i] = ((sum - distances[i]) / sum);
            }
            sum = weights.Sum();

            for (int i = 0; i < distances.Length; i++)
            {
                weights[i] /= sum;
            }

            // Generate new DH table by combining af all DH tables according their weights in current position
            for (int i = 0; i < resultDH.Length; i++)
            {
                double newVal = 0;
                for (int j = 0; j < weights.Length; j++)
                {
                    newVal += ZonesDHtables[j][i] * weights[j];
                }
                resultDH[i] = newVal;
            }

            return resultDH;
        }

        double getjointDistance(double[] jointsA, double[] jointsB)
        {
            double sums = 0;
            int[] highWeigth = new int[] { 3, 4, 5};
            for (int i = 0; i < jointsA.Length; i++)
            {
                sums += Math.Pow(jointsA[i] - jointsB[i + 1], 2);
                //if (highWeigth.Contains(i))
                //{
                //    sums += Math.Pow(jointsA[i] - jointsB[i + 1], 2);
                //}
                //else
                //{
                //    sums += Math.Pow(jointsA[i] - jointsB[i + 1], 2) / 2;
                //}
            }

            return Math.Sqrt(sums);
        }

        double distance(double x1, double y1, double z1, double x2, double y2, double z2)
        {
            double a = (x2 - x1);
            double b = (y2 - y1);
            double c = (z2 - z1);

            return (double)Math.Sqrt((a * a) + (b * b) + (c * c));
        }

        void store()
        {
            {
                double[] values = { 75, 200 };
                double[] distances = { 1, 5 };
                double sum = distances.Sum();
                double[] ratios = new double[distances.Length];

                for (int i = 0; i < distances.Length; i++)
                {
                    ratios[i] = (sum - distances[i]) / sum;
                }

                sum = ratios.Sum();

                for (int i = 0; i < distances.Length; i++)
                {
                    ratios[i] /= sum;
                }

                sum = ratios.Sum();

                var test = values[0] * ratios[0] + values[1] * ratios[1];
            }
        }
    }
}
