using Caliburn.Micro;
using System;
using System.Linq;
using System.Windows.Input;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;
using VisualComponents.UX.Shared.MenuCommands.CollisionDetection;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using vcPluginHelioLic;

namespace SnapNMS_Namespace
{
    public class SnapActionsNMS : INotifyPropertyChanged
    {
        IMessageService _MessageService;
        IApplication _Application;
        ITeachContext _TeachContext;
        SnapAction _snapAction;
        IDetectorManager _DetectorManager;
        ISimulation _Simulation;

        ISimComponent Tracker = null;
        IProperty TrackNow = null;
        IProperty TargetLed = null;
        IProperty TargetSide = null;
        IRealSignal I__AngleSignal = null;

        double originCollTollerance = 10;
        public ISimComponent LastUsedTracker, Current_TSGeometry;
        public int LastConfigNum = -1, NumOfTests;
        public Matrix LastSnaped, OrigTSGeometryPos, TCP_Helper;
        public IMotionTarget DefaultTarget;
        public bool Activated = false, EventWindowActivated = false, ToolTmac = false, ForcedInterrupt = false;
        public double X_LiveAdjust, Y_LiveAdjust, Z_LiveAdjust;
        public IList<IVisualObject> RebuildList = new List<IVisualObject>();
        public IInputElement InputElement;
        public IList<ISimNode> JointList;
        public double SpendTimeForCollTests = 0;
        bool ManualSnap = false;

        List<Matrix> TeachedTargets = null;
        int FastTeached = 0;

        public SnapActionsNMS(IApplication application, ITeachContext TeachContext, IDetectorManager detectorManager, IMessageService messageService)
        {
            _MessageService = messageService;
            _Application = application;
            _TeachContext = TeachContext;
            _DetectorManager = detectorManager;
            _Simulation = _Application.Simulation;


            StatusBar = "StatusBar";
            TeachedTargets = new List<Matrix>();
            FastTeached = 0;
        }

        public void ToggleSnap(bool manualSnap)
        {
            if (HelioLic.isAllowed("N6001-0001") == 0) { return; }
            var test = FuckYouHXGN.Brrr(5);

            ManualSnap = manualSnap;

            // Get robot and details
            IRobot robot = _TeachContext.ActiveRobot;
            if (robot == null)
            {
                StatusBar = "Please select robot program first";
                return;
            }
            if (robot.RobotController == null)
            {
                //print("Please select robot program first");
                StatusBar = "Please select robot program first";
                return;
            }
            

            LastUsedTracker = getCurrentTracker(robot);
            if (LastUsedTracker == null)
            {
                return;
            }

            if (Activated == false)
            {
                activateDetectors();

                IStatementScope activeScope = _TeachContext.ActiveScope;
                IRobotController robotController = robot.RobotController;

                Current_TSGeometry = getTSGeometry(robot.Component.RootNode);

                if (Current_TSGeometry == null) 
                {
                    print("Missing TCP_helper frame in T-Scan geometry");
                    return; 
                }
                Current_TSGeometry.IsLocked = false;
                OrigTSGeometryPos = Current_TSGeometry.TransformationInReference;

                JointList = new List<ISimNode>();

                TCP_Helper = getTCP_Helper();

                DefaultTarget = robotController.GetMotionTester().CurrentTarget;

                // Activate Snap tool
                if (ManualSnap)
                {
                    SnapType[] types = new SnapType[] { SnapType.Face };
                    _snapAction = new SnapAction(true, SnapOptionsContext.NormalSnap);
                    _snapAction.ActionSnapOptions = types;
                    _snapAction.WantSurfaceNormal = true;
                    _snapAction.ContinuousMode = true;
                    _snapAction.PublishResult = false;
                    _snapAction.DontSnapToSelection = true;
                    _snapAction.ContinuousResultMouseMove += ContinuousSnap;

                    _Simulation.SimulationStarted += DeactivationEvents;
                    _Simulation.SimulationReset += DeactivationEvents;
                }


                Activated = true;
                StatusBar = "Activated";

                LastConfigNum = -1;
                NumOfTests = 0;

                X_LiveAdjust = 0;
                Y_LiveAdjust = 0;
                Z_LiveAdjust = 0;

            }
            else
            {
                Activated = false;

                if (ManualSnap)
                {
                    _snapAction.ContinuousResultMouseMove -= ContinuousSnap;

                    _Simulation.SimulationStarted -= DeactivationEvents;
                    _Simulation.SimulationReset -= DeactivationEvents;
                }


                IRobotController robotController = robot.RobotController;
                IMotionTester motionTester = robotController.GetMotionTester();
                motionTester.CurrentTarget = DefaultTarget;
                motionTester.Dispose();

                foreach (ICollisionDetector collisionDetector in _DetectorManager.AllDetectors)
                {

                    if (collisionDetector.Name != "Selection vs World")
                    {
                        collisionDetector.IsActive = false;
                    }
                    originCollTollerance = collisionDetector.Tolerance;
                }

                StatusBar = "Dectivated";

                if (Current_TSGeometry != null)
                {
                    Current_TSGeometry.TransformationInReference = OrigTSGeometryPos;
                    Current_TSGeometry.Update();
                    foreach (IFeature feature in Current_TSGeometry.Features)
                    {
                        if ((feature.Name == "Body") || (feature.Name == "TScan-Protection"))
                        {
                            feature.IsVisible = true;
                        }
                    }
                    Current_TSGeometry.IsLocked = false;
                }

                _Application.ActiveWindow.Render();
                Tracker = null;
            }
        }

        void showToolOnTarget(IRobot robot, Matrix target)
        {
            Current_TSGeometry.TransformationInWorld = robot.RobotController.CurrentBase.TransformationInWorld * target * TCP_Helper;

            Current_TSGeometry.Update();
        }


        #region Teach procedure

        public void ManualScanLin(IRobot robot, Matrix target)
        {
            //List<(double, int, int, Matrix)> searchResults = new List<(double, int, int, Matrix)>() { (0, 1, 7, target) };
            //(double, int, int, IMotionTarget) collFreeResult = collisionTests(robot, searchResults, LastTeachedLIN);

            IRobotController robotController = robot.RobotController;
            IMotionTester motionTester = robotController.GetMotionTester();

            IMotionTarget resultMotion = findColFreeConfig(motionTester, target);

            motionTester.Dispose();
            // Create LIN statement
            if (resultMotion != null)
            {
                ILinearMotionStatement TeachedLIN = TeachLIN(robot, resultMotion);
            }

            robotController.GetMotionTester().CurrentTarget = DefaultTarget;
            robotController.GetMotionTester().Dispose();
            _Application.ActiveWindow.Render();
        }

        public (double, int, int, ILinearMotionStatement) FindScanLIN(IRobot robot, Matrix target, int K_AngleStep)
        {
            //Stopwatch stopwatch = new Stopwatch();
            if (ForcedInterrupt) { return (0,0,0,null); }

            IRobotController robotController = robot.RobotController;
            ILinearMotionStatement TeachedLIN = null;

            IRenderService renderService = IoC.Get<IRenderService>();
            renderService.IsRenderingEnabled = true;

            if (TeachedTargets.Count > 0)
            {
                Matrix testedTarget = new Matrix();
                List<(double, int, int, Matrix)> preTestResults = new List<(double, int, int, Matrix)>();

                for (int i = TeachedTargets.Count - 1; i >= 0; i--)
                {
                    double angleA = Vector3.AngleBetween(target.GetA(), TeachedTargets[i].GetA());

                    if ((angleA * (180.0 / 3.1415)) < 15)
                    {
                        showRobotBody(robot, true);

                        testedTarget.SetP(target.GetP());
                        testedTarget.SetWPR(TeachedTargets[i].GetWPR());

                        showToolOnTarget(robot, testedTarget);
                        var trackData = getTrakerData();

                        if (trackData.Item1 < 45)
                        {
                            preTestResults.Add((trackData.Item1 * angleA / 0.26, trackData.Item2, trackData.Item3, testedTarget));
                        }
                    }
                    showRobotBody(robot, false);

                    if (preTestResults.Count > 0)
                    {
                        preTestResults = preTestResults.OrderBy(x => x.Item1).ToList();
                        var collResult = collisionTests(robot, preTestResults);

                        // Create LIN statement
                        if (collResult.Item4 != null)
                        {
                            TeachedLIN = TeachLIN(robot, collResult.Item4);

                            FastTeached++;
                            _MessageService.ClearMessages();
                            print($"Fast teached targets: {FastTeached}");

                            robotController.GetMotionTester().CurrentTarget = DefaultTarget;
                            robotController.GetMotionTester().Dispose();
                            renderService.IsRenderingEnabled = true;

                            showRobotBody(robot, true);
                            // (Visibilty, lockedSide, visibleLeds, LIN)
                            return (collResult.Item1, collResult.Item2, collResult.Item3, TeachedLIN);
                        }
                    }
                }
            }

            double[] xzRotations = new double[] { 5, 10 };

            showRobotBody(robot, false);
            List<(double, int, int, Matrix)> searchResults = searchVisibility(robot, target, K_AngleStep, xzRotations);
            showRobotBody(robot, true);

            //return geometry back to robot arm
            Current_TSGeometry.TransformationInReference = OrigTSGeometryPos;

            (double, int, int, IMotionTarget) collFreeResult = collisionTests(robot, searchResults);

            if (collFreeResult.Item4 == null && false)
            {
                xzRotations = new double[] { 20, 40 };

                showRobotBody(robot, false);
                searchResults = searchVisibility(robot, target, K_AngleStep, xzRotations, true);
                showRobotBody(robot, true);


                if (false)
                {
                    IMotionTester mtstr = robot.RobotController.GetMotionTester();
                    IMotionTarget mt = mtstr.CurrentTarget;
                    ILinearMotionStatement lin = null;
                    foreach (var result in searchResults)
                    {
                        mt.TargetMatrix = result.Item4;
                        lin = TeachLIN(robot, mt);
                    }
                    mtstr.Dispose();
                    return (0, 0, 0, lin);
                }


                //return geometry back to robot arm
                Current_TSGeometry.TransformationInReference = OrigTSGeometryPos;

                collFreeResult = collisionTests(robot, searchResults);

            }

            if (collFreeResult.Item4 == null)
            {
                robotController.GetMotionTester().CurrentTarget = DefaultTarget;
                robotController.GetMotionTester().Dispose();
                renderService.IsRenderingEnabled = true;
                return (0, 0, 0, TeachedLIN);
            }

            // Create LIN statement
            if (collFreeResult.Item4 != null)
            {
                TeachedLIN = TeachLIN(robot, collFreeResult.Item4);
                TeachedTargets.Add(collFreeResult.Item4.TargetMatrix);
            }

            robotController.GetMotionTester().CurrentTarget = DefaultTarget;
            robotController.GetMotionTester().Dispose();
            renderService.IsRenderingEnabled = true;

            // (Visibilty, lockedSide, visibleLeds, LIN)
            return (collFreeResult.Item1, collFreeResult.Item2, collFreeResult.Item3, TeachedLIN);
        }

        void showRobotBody(IRobot robot, bool show)
        {
            ISimComponent robComponent = robot.Component;
            //Current_TSGeometry

            robComponent.BeginChangeBatch();
            foreach (ISimNode node in robComponent.Nodes)
            {
                foreach(IFeature feature in node.Features)
                {
                    feature.IsVisible = show;
                }
            }
            robComponent.EndChangeBatch();
        }

        public ILinearMotionStatement TeachLIN(IRobot robot, IMotionTarget motionTarget)
        {
            /// <summary>Method for teaching Linear statement.</summary>
            /// <param name="robot">Tested robot</param>
            /// <param name="target">Snapped target</param>
            /// <returns> Teached ILinearMotionStatement </returns>

            IRobotController robotController = robot.RobotController;
            ILinearMotionStatement statementLIN, statementUp;

            int index = _TeachContext.ActiveStatementIndex;
            IStatementScope activeScope = _TeachContext.ActiveScope;
            statementLIN = activeScope.AddStatement<ILinearMotionStatement>(index + 1);
            statementLIN.Base = robotController.CurrentBase;
            statementLIN.Tool = robotController.CurrentTool;

            if (index != -1)
            {
                if (activeScope.GetStatement(index).ApiType.Name == "ILinearMotionStatement")
                {
                    statementUp = (ILinearMotionStatement)activeScope.GetStatement(index);
                    statementLIN.MaxSpeed = statementUp.MaxSpeed;
                    statementLIN.AccuracyMethod = statementUp.AccuracyMethod;
                    statementLIN.AccuracyValue = statementUp.AccuracyValue = 10;
                }
                else
                {
                    statementLIN.MaxSpeed = 250;
                    statementLIN.AccuracyMethod = AccuracyMethod.Distance;
                    statementLIN.AccuracyValue = 10;
                }
            }
            else
            {
                statementLIN.MaxSpeed = 250;
                statementLIN.AccuracyMethod = AccuracyMethod.Distance;
                statementLIN.AccuracyValue = 10;
            }

            // Put target to Statement
            statementLIN.Target = motionTarget.TargetMatrix;

            IPositionFrame pos = statementLIN.Positions.ElementAt(0);

            for (int extAxisIndex = 0; extAxisIndex < 10; extAxisIndex++)
            {
                try
                {
                    var extaxis = pos.GetProperty("E" + extAxisIndex);
                    if (extaxis != null)
                    {
                        double extAxisValue = motionTarget.GetExternalJointValues()[extAxisIndex-1];
                        extaxis.SetValue("Value", extAxisValue);
                    }
                }
                catch { }
            }

            // Set configuration - set last used config if possible
            List<int> goodConfigs = new List<int>();
            for (int confIndex = 0; confIndex < motionTarget.ConfigurationCount; confIndex++)
            {
                if (motionTarget.GetConfigurationWarnings(confIndex) == 0)
                {
                    goodConfigs.Add(confIndex);
                }
            }
            if (goodConfigs.Count != 0)
            {
                if (goodConfigs.Contains(LastConfigNum))
                {
                    statementLIN.Positions[0].Configuration = LastConfigNum;
                }
                else
                {
                    statementLIN.Positions[0].Configuration = goodConfigs[0];
                }
            }
            else
            {
                statementLIN.Positions[0].Configuration = LastConfigNum;
            }
            LastConfigNum = statementLIN.Positions[0].Configuration;

            //StatusBar = "LIN P" + (index + 1) + " Config[" + LastConfigNum + "] teached";

            // Set new active statement
            _TeachContext.SetActiveStatement(activeScope, index + 1);

            return statementLIN;
        }

        #endregion

        #region Target adjusting

        Matrix transformPosition(IRobot robot, Vector3 snapPos, Vector3 snapNor)
        {
            /// <summary>Method for snapped position into current robot Base Frame</summary>
            /// <param name="robot">Tested robot</param>
            /// <param name="snapPos">Snapped position Vector3</param>
            /// <param name="snapNor">Snapped surface normal Vector3</param>
            /// <returns> Transformed Matrix position in active robot base frame </returns>

            IRobotController robotController = robot.RobotController;

            double normalX = 0, normalY = 0, normalZ = 0;
            // Create temp_target
            Matrix temp_target = new Matrix();

            Matrix tcp_mtx = robotController.ToolCenterPoint;

            // Move target_mtx to snap position
            temp_target.Px = snapPos.X;
            temp_target.Py = snapPos.Y;
            temp_target.Pz = snapPos.Z;

            // First step, align with TCP in Base
            temp_target.SetN(tcp_mtx.GetN());
            temp_target.SetO(tcp_mtx.GetO());
            temp_target.SetA(tcp_mtx.GetA());

            // Align with surface normal
            if (snapNor != null)
            {
                Vector3 snapNormal = (Vector3)snapNor;
                normalX = snapNormal.X;
                normalY = snapNormal.Y;
                normalZ = snapNormal.Z;
                temp_target.SetIJK(normalX, normalY, normalZ);
            }


            // get Base mtx in world coord system
            if (robotController.CurrentBase == null)
            {
                _snapAction.ContinuousResultMouseMove -= ContinuousSnap;
                _snapAction.EndAction(true, true);

                //StatusBar = "Choose Robot Base before snap!";

                //SnapSwitch();
                return new Matrix();
            }
            Matrix base_mtx = robotController.CurrentBase.TransformationInWorld;

            // Put target to Base coord system
            Matrix target = base_mtx.Inverse() * temp_target;


            return target;
        }

        List<(double, int, int, Matrix)> searchVisibility(IRobot robot, Matrix testTarget, int ZAngleStep, double[] xzRotations, bool firstZ = true)
        {
            Matrix basedOnMTX = testTarget;
            Matrix originTarget = testTarget;

            List<(double, int, int, Matrix)> testedResults = new List<(double, int, int, Matrix)>();

            if (Tracker == null)
            {
                Tracker = getCurrentTracker(robot);
                if (Tracker == null)
                {
                    ToggleSnap(true);
                    return new List<(double, int, int, Matrix)>();
                }

                TrackNow = null;
                TargetLed = null;
                TargetSide = null;
                I__AngleSignal = null;
                try
                {
                    TrackNow = Tracker.GetProperty("TrackNow");
                    TargetLed = Tracker.GetProperty("TargetLed");
                    TargetSide = Tracker.GetProperty("TargetSide");
                    I__AngleSignal = Tracker.FindBehavior("I__AngleSignal") as IRealSignal;

                    if (TrackNow == null || TargetLed == null || TargetSide == null || I__AngleSignal == null)
                    {
                        Tracker = null;
                        System.Console.Beep(300, 300);
                        System.Console.Beep(300, 300);
                        return testedResults;
                    }
                }
                catch
                {
                    Tracker = null;
                    System.Console.Beep(300, 300);
                    System.Console.Beep(300, 300);
                    return testedResults;
                }
            }

            (double, int, int) trackData;

            int ZItterations = 360 / ZAngleStep;


            List<Matrix> zResults = new List<Matrix>();
            bool thisIsOriginal = true;
            double multiplyer = 1;

            List<double> teachedAngles = new List<double>();

            foreach (double xzAng in xzRotations)
            {
                for (int j = 0; j <= 8; j++)
                {
                    if (firstZ)
                    {
                        // Test in Z rotations
                        for (int itterZ = 0; itterZ < ZItterations; itterZ++)
                        {
                            showToolOnTarget(robot, testTarget);
                            trackData = getTrakerData();

                            if (trackData.Item1 < 30)
                            {
                                zResults.Add(testTarget);
                                if (thisIsOriginal)
                                {
                                    testedResults.Add((0, trackData.Item2, trackData.Item3, testTarget));
                                }
                                else
                                {
                                    //double testDelta = Vector3.AngleBetween(testTarget.GetN(), basedOnMTX.GetN()) + Vector3.AngleBetween(testTarget.GetO(), basedOnMTX.GetO()) + 1; //getDifference(testTarget, basedOnMTX) + 1;

                                    double testDelta = Vector3.AngleBetween(testTarget.GetA(), originTarget.GetA()) / 0.26; // If angle is less than 15°, do not decrease score
                                    teachedAngles.Add(testDelta * 180 / 3.1415);

                                    if (testDelta < 1) { testDelta = 1; }
                                    testedResults.Add((trackData.Item1 * multiplyer * testDelta, trackData.Item2, trackData.Item3, testTarget));

                                }
                            }

                            thisIsOriginal = false;
                            testTarget.RotateAroundZ(ZAngleStep);
                        }
                    }

                    testTarget = originTarget;
                    switch (j)
                    {
                        case 0:
                            testTarget.RotateAroundX(xzAng);
                            break;
                        case 1:
                            testTarget.RotateAroundY(-xzAng);
                            break;
                        case 2:
                            testTarget.RotateAroundX(-xzAng);
                            break;
                        case 3:
                            testTarget.RotateAroundX(-xzAng);
                            break;
                        case 4:
                            testTarget.RotateAroundY(xzAng);
                            break;
                        case 5:
                            testTarget.RotateAroundY(xzAng);
                            break;
                        case 6:
                            testTarget.RotateAroundX(xzAng);
                            break;
                        case 7:
                            testTarget.RotateAroundX(xzAng);
                            break;
                    }

                    if (!firstZ)
                    {
                        // Test in Z rotations
                        for (int itterZ = 0; itterZ < ZItterations; itterZ++)
                        {
                            showToolOnTarget(robot, testTarget);
                            trackData = getTrakerData();

                            if (trackData.Item1 < 30)
                            {
                                zResults.Add(testTarget);
                                if (thisIsOriginal)
                                {
                                    testedResults.Add((0, trackData.Item2, trackData.Item3, testTarget));
                                }
                                else
                                {
                                    //double testDelta = Vector3.AngleBetween(testTarget.GetN(), basedOnMTX.GetN()) + Vector3.AngleBetween(testTarget.GetO(), basedOnMTX.GetO()) + 1; //getDifference(testTarget, basedOnMTX) + 1;

                                    double testDelta = Vector3.AngleBetween(testTarget.GetA(), originTarget.GetA()) / 0.26; // If angle is less than 15°, do not decrease score
                                    teachedAngles.Add(testDelta * 180 / 3.1415);

                                    if (testDelta < 1) { testDelta = 1; }
                                    testedResults.Add((trackData.Item1 * multiplyer * testDelta, trackData.Item2, trackData.Item3, testTarget));

                                }
                            }

                            thisIsOriginal = false;
                            testTarget.RotateAroundZ(ZAngleStep);
                        }
                    }
                }
            }



            testedResults = testedResults.OrderBy(x => x.Item1).ToList();

            return testedResults;
        }

        
        List<Matrix> sortingByAngle(List<Matrix> inputList, Matrix startPoint)
        {
            List<Matrix> sorted = new List<Matrix>();

            if (inputList.Count < 1) { return null; }

            double closestDist = double.MaxValue;
            Matrix referenceMTX = startPoint;
            Matrix closestTarget = inputList[0];

            while (300 > sorted.Count)
            {
                for (int index = 0; index < inputList.Count; index++)
                {
                    Matrix testTarget = inputList[index];
                    if (sorted.Contains(testTarget))
                    {
                        continue;
                    }

                    double distance = getDifference(referenceMTX, testTarget);

                    if (distance < closestDist)
                    {
                        closestDist = distance;
                        closestTarget = testTarget;
                    }
                }
                sorted.Add(closestTarget);
                closestDist = double.MaxValue;
                //referenceMTX = sorted.Last<(double, int, int, Matrix)>().Item4;
            }
            return sorted;
        }

        #endregion

        #region Robot adjusting

        bool isMotionReachable(IMotionTarget motionTarget)
        {
            bool noWarnings = false;

            Matrix origTarget = motionTarget.TargetMatrix;

            motionTarget = solveMotionLimits(motionTarget);

            var allJointValues = motionTarget.GetAllJointValues();
            motionTarget.TargetMatrix = origTarget;
            allJointValues = motionTarget.GetAllJointValues();

            for (int confIndex = 0; confIndex < motionTarget.ConfigurationCount; confIndex++)
            {
                if (motionTarget.GetConfigurationWarnings(confIndex) == 0)
                {
                    if (motionTarget.CurrentConfiguration == confIndex) { noWarnings = true; } // True only if is currently used config available
                }
            }

            allJointValues = motionTarget.GetAllJointValues();
            return noWarnings;
        }

        IMotionTarget findColFreeVisibleConfig(IMotionTester motionTester, Matrix target)
        {
            bool targetReachable = false;
            double toolVisibility = double.MaxValue;

            IRobotController robotController = motionTester.CurrentTarget.RobotController;

            IMotionTarget motionTarget = motionTester.CurrentTarget;
            IMotionTarget originalTarget = motionTarget;
            motionTarget.TargetMatrix = target;
            motionTester.CurrentTarget = motionTarget;

            // Check and solve joint limits
            //motionTarget = solveMotionLimits(motionTarget);

            Matrix tcpPosMTX = motionTester.CurrentTarget.TargetMatrix;
            double distance = getDistanceTarget_TCP(tcpPosMTX, target);
            bool inCollision = true;
            if (distance == 0)
            {
                targetReachable = isMotionReachable(motionTarget);
                toolVisibility = getTrakerData().Item1;
                inCollision = isInCollision();
                var allJointValues = motionTarget.GetAllJointValues();
            }

            if ((inCollision || (robotController.ExternalJoints.Count > 0) && !targetReachable) && motionTarget.RobotPositioner != null)
            {
                IList<IJoint> extAxes = motionTarget.RobotPositioner.Joints;

                int incrementExAxis = 500;
                bool minReached = false;
                bool maxReached = false;
                List<double> usedExtValues = new List<double>();
                while ((distance != 0 || !targetReachable) || (toolVisibility > 45 || inCollision))
                {
                    if (ForcedInterrupt) { break; }

                    if (minReached && maxReached) { break; }

                    extAxes[0].Value += incrementExAxis;

                    if (usedExtValues.Contains(extAxes[0].Value))
                    {
                        continue;
                    }

                    if (extAxes[0].Value < extAxes[0].MinValue && incrementExAxis < 0)
                    {
                        incrementExAxis *= -1;
                        minReached = true;
                        continue;
                    }

                    if (extAxes[0].Value > extAxes[0].MaxValue && incrementExAxis > 0)
                    {
                        incrementExAxis *= -1;
                        maxReached = true;
                        continue;
                    }

                    motionTarget = motionTester.CurrentTarget;
                    motionTarget.TargetMatrix = target;
                    motionTester.CurrentTarget = motionTarget;

                    tcpPosMTX = motionTester.CurrentTarget.TargetMatrix;
                    distance = getDistanceTarget_TCP(tcpPosMTX, target);

                    if (distance == 0)
                    {
                        toolVisibility = getTrakerData().Item1;
                        targetReachable = isMotionReachable(motionTarget);
                        if (targetReachable)
                        {
                            motionTester.CurrentTarget = motionTarget;
                            inCollision = isInCollision();
                        }
                    }
                    usedExtValues.Add(extAxes[0].Value);
                }

            }
            IMotionTarget resultTarget = motionTarget;// motionTester.CurrentTarget; <=====

            // vráť robota do pôvodnej polohy
            motionTester.CurrentTarget = originalTarget;

            if (ForcedInterrupt) { return null; }
            if (toolVisibility < 45 && targetReachable && !inCollision)
            {
                return resultTarget;
            }
            else
            {
                return null;
            }
        }

        IMotionTarget findColFreeConfig(IMotionTester motionTester, Matrix target)
        {
            double basedOn_ExtAxis = double.NaN;

            bool targetReachable = false;

            IRobotController robotController = motionTester.CurrentTarget.RobotController;

            IMotionTarget motionTarget = motionTester.CurrentTarget;
            IMotionTarget originalTarget = motionTarget;
            motionTarget.TargetMatrix = target;
            motionTester.CurrentTarget = motionTarget;

            // Check and solve joint limits
            //motionTarget = solveMotionLimits(motionTarget);

            Matrix tcpPosMTX = motionTester.CurrentTarget.TargetMatrix;
            double distance = getDistanceTarget_TCP(tcpPosMTX, target);
            bool inCollision = true;
            if (distance == 0)
            {
                targetReachable = isMotionReachable(motionTarget);
                inCollision = isInCollision();
                //var allJointValues = motionTarget.GetAllJointValues();
            }

            if ((inCollision || !targetReachable) && motionTarget.RobotPositioner != null)
            {
                IList<IJoint> extAxes = motionTarget.RobotPositioner.Joints;

                if (!double.IsNaN(basedOn_ExtAxis))
                {
                    extAxes[0].Value = basedOn_ExtAxis;
                }

                int incrementExAxis = 500;
                bool minReached = false;
                bool maxReached = false;
                List<double> usedExtValues = new List<double>();
                while ((distance != 0 || !targetReachable) || inCollision)
                {
                    if (ForcedInterrupt) { break; }
                    if (minReached && maxReached) { break; }

                    extAxes[0].Value += incrementExAxis;

                    if (usedExtValues.Contains(extAxes[0].Value))
                    {
                        continue;
                    }

                    if (extAxes[0].Value < extAxes[0].MinValue && incrementExAxis < 0)
                    {
                        incrementExAxis *= -1;
                        minReached = true;
                        continue;
                    }

                    if (extAxes[0].Value > extAxes[0].MaxValue && incrementExAxis > 0)
                    {
                        incrementExAxis *= -1;
                        maxReached = true;
                        continue;
                    }

                    motionTarget = motionTester.CurrentTarget;
                    motionTarget.TargetMatrix = target;
                    motionTester.CurrentTarget = motionTarget;

                    tcpPosMTX = motionTester.CurrentTarget.TargetMatrix;
                    distance = getDistanceTarget_TCP(tcpPosMTX, target);

                    if (distance == 0)
                    {
                        targetReachable = isMotionReachable(motionTarget);
                        if (targetReachable)
                        {
                            motionTester.CurrentTarget = motionTarget;
                            inCollision = isInCollision();
                        }
                    }
                    usedExtValues.Add(extAxes[0].Value);
                }

            }
            IMotionTarget resultTarget = motionTarget;// motionTester.CurrentTarget; <=====

            // vráť robota do pôvodnej polohy
            motionTester.CurrentTarget = originalTarget;

            if (ForcedInterrupt) { return null; }
            if (targetReachable && !inCollision)
            {
                return resultTarget;
            }
            else
            {
                return null;
            }
        }

        IMotionTarget solveMotionLimits(IMotionTarget motionTarget)
        {
            double[] allJointsVals = motionTarget.GetAllJointValues();
            IList<IJoint> allJoints = motionTarget.RobotController.Joints;
            int indexJoint = 0;
            foreach (IJoint joint in allJoints)
            {
                if (joint.Type == JointType.Translational)
                {
                    if (allJointsVals[indexJoint] < joint.MinValue) { allJointsVals[indexJoint] = joint.MinValue; }
                    if (allJointsVals[indexJoint] > joint.MaxValue) { allJointsVals[indexJoint] = joint.MaxValue; }
                }
                else if (joint.Type == JointType.Rotational)
                {
                    if (allJointsVals[indexJoint] < joint.MinValue && allJointsVals[indexJoint] + 360 < joint.MaxValue)
                    {
                        allJointsVals[indexJoint] += 360; 
                    }
                    if (allJointsVals[indexJoint] > joint.MaxValue && allJointsVals[indexJoint] - 360 > joint.MinValue)
                    {
                        allJointsVals[indexJoint] -= 360;
                    }
                }

                indexJoint++;
            }
            motionTarget.SetAllJointValues(allJointsVals);

            return motionTarget;
        }

        #endregion

        #region Collision test

        void activateDetectors()
        {
            _DetectorManager.IsActive = true;
            _DetectorManager.ReConfigureDetectors();
            _DetectorManager.ClearAllCollisionHighlighting(false);
            foreach (ICollisionDetector collisionDetector in _DetectorManager.AllDetectors)
            {

                if (collisionDetector.Name != "Selection vs World")
                {
                    collisionDetector.IsActive = false;
                }
                collisionDetector.DetectFirstCollisionOnly = true;
                collisionDetector.IgnoreInvisible = true;
                collisionDetector.Highlight = true;
                collisionDetector.ClearCollisionHighlighting();

                originCollTollerance = collisionDetector.Tolerance;
                if (collisionDetector.Tolerance < 10) { collisionDetector.Tolerance = 10; }
                collisionDetector.Tolerance += 20;
            }
            //_Simulation.Reset();
            //_Application.ActiveWindow.Render();
        }


        bool isInCollision()
        {
            foreach (ICollisionDetector collisionDetector in _DetectorManager.AllDetectors)
            {
                ForcedInterrupt = Keyboard.IsKeyDown(Key.Escape);

                if (collisionDetector.Name != "Selection vs World")
                {
                    collisionDetector.IsActive = true;
                    //collisionDetector.IgnoreInvisible = true;
                    //collisionDetector.ClearCollisionHighlighting();
                    //collisionDetector.TestOneCollision();
                    if (collisionDetector.TestOneCollision().Count > 0)
                    {
                        //var result = collisionDetector.TestOneCollision();
                        //collisionDetector.IsActive = false;
                        return true;
                    }
                }
            }

            /*
            if (_DetectorManager.AllDetectors[1].TestOneCollision().Count > 0)
            {
                //var result = collisionDetector.TestOneCollision();
                return true;
            }
            */
            return false;
        }

        (double, int, int, IMotionTarget) collisionTests(IRobot robot, List<(double, int, int, Matrix)> targetList)
        {
            /// <summary>Method is testing all targets from input list.</summary>
            /// <param name="robot">Tested robot</param>
            /// <param name="targetList">List of targets for testing</param>
            /// <returns> First collision free target as Matrix. </returns>
            /// <returns> New Matrix if all targets are in collision. </returns>
            /// 
            /// Uses method:
            /// <see cref="SolveConfig(IRobot, Matrix)" />
            /// <see cref="isInCollision" />

            if (ForcedInterrupt) { return (0,0,0,null); }

            
            IRobotController robotController = robot.RobotController;
            IMotionTester motionTester = robotController.GetMotionTester();
            IMotionTarget currentTarget = motionTester.CurrentTarget;

            // Ak si získal jointové hodnoty z predchádzajúceho pohybu v rutine, aplikuj ich

            List<double> initJoints = new List<double>();
            foreach (IJoint joint in robotController.Joints)
            {
                initJoints.Add(joint.InitialValue);
            }
            foreach (IJoint exJoint in robotController.ExternalJoints)
            {
                initJoints.Add(exJoint.InitialValue);
            }
            currentTarget.SetAllJointValues(initJoints);

            motionTester.CurrentTarget = currentTarget;


            

            foreach ((double, int, int, Matrix) item in targetList)
            {
                IMotionTarget configrdTarget = findColFreeVisibleConfig(motionTester, item.Item4);
                if (configrdTarget == null)
                {
                    continue;
                }

                if (item.Item4.GetP() == configrdTarget.TargetMatrix.GetP())
                {
                    motionTester.CurrentTarget = configrdTarget;
                    (double, int, int) trackData = getTrakerData();
                    if (trackData.Item1 > 45)
                    {
                        continue;
                    }

                    motionTester.Dispose();
                    return (trackData.Item1, item.Item2, item.Item3, configrdTarget);

                }
            }

            motionTester.Dispose();

            return (0, 0, 0, null);
        }

        List<(double, int, int, Matrix)> preSortingByAngle(List<(double, int, int, Matrix)> inputList, Matrix startPoint)
        {
            List<(double, int, int, Matrix)> sorted = new List<(double, int, int, Matrix)>();

            if (inputList.Count < 1) { return null; }

            double closestDist = double.MaxValue;
            Matrix referenceMTX = startPoint;
            (double, int, int, Matrix) closestTarget = inputList[0];

            while (sorted.Count < inputList.Count)
            {
                for (int index = 0; index < inputList.Count; index++)
                {
                    (double, int, int, Matrix) testTarget = inputList[index];
                    if (referenceMTX == testTarget.Item4) { continue; }
                    if (sorted.Contains(testTarget))
                    {
                        continue;
                    }

                    double distance = getDifference(referenceMTX, testTarget.Item4);

                    if (distance < closestDist)
                    {
                        closestDist = distance;
                        closestTarget = testTarget;
                    }
                }
                sorted.Add(closestTarget);
                closestDist = double.MaxValue;
                //referenceMTX = sorted.Last<(double, int, int, Matrix)>().Item4;
            }
            return sorted;
        }

        double getDifference(Matrix A, Matrix B)
        {
            double deltaX = B.Px - A.Px;
            double deltaY = B.Py - A.Py;
            double deltaZ = B.Pz - A.Pz;


            double deltaI = B.Ax - A.Ax;
            double deltaJ = B.Ay - A.Ay;
            double deltaK = B.Az - A.Az;

            //return ((deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) * 0.00001) * ((deltaI * deltaI + deltaJ * deltaJ + deltaK * deltaK) + 1);
            return (deltaI * deltaI + deltaJ * deltaJ + deltaK * deltaK);
        }

        #endregion

        #region Events
        public void KeyEvents(ActionExecutionContext context)
        {
            /// <summary>Method for catching keyboard events. Used for keyboard shortcuts.>

            double local_Z_liveAdjust = 0, local_Y_liveAdjust = 0, local_X_liveAdjust = 0;
            InputElement = Keyboard.FocusedElement;
            if (!EventWindowActivated)
            {
                context.Source.LostKeyboardFocus += WindowEvent;
                EventWindowActivated = true;
            }

            if ((Key)context.EventArgs.GetValue("Key") == Key.Escape)
            {
                if (Activated)
                {
                    ToggleSnap(true);
                }
            }


            if ((Key)context.EventArgs.GetValue("Key") == Key.LeftCtrl)
            {
                if (_TeachContext.ActiveRobot == null)
                {
                    return;
                }
                IRobotController robotController = _TeachContext.ActiveRobot.RobotController;
                robotController.GetMotionTester().Dispose();
                
                //FindScanLIN(_TeachContext.ActiveRobot, LastSnaped, 5, 5, 5, false);

                ManualScanLin(_TeachContext.ActiveRobot, LastSnaped);

                //JustTeach(_TeachContext.ActiveRobot, lastSnaped);
                return;
            }

            // Keyboard adjust
            if ((Key)context.EventArgs.GetValue("Key") == Key.F)
            {
                if (Activated == false)
                {
                    return;
                }

                foreach (IFeature feature in Current_TSGeometry.Features)
                {
                    if ((feature.Name == "Body") || (feature.Name == "TScan-Protection"))
                    {
                        feature.IsVisible = true;
                    }
                }
            }


            else if ((Key)context.EventArgs.GetValue("Key") == Key.Q)
            {
                Z_LiveAdjust -= 10;
                local_Z_liveAdjust -= 10;
            }

            else if ((Key)context.EventArgs.GetValue("Key") == Key.E)
            {
                Z_LiveAdjust += 10;
                local_Z_liveAdjust += 10;
            }

            else if ((Key)context.EventArgs.GetValue("Key") == Key.S)
            {
                X_LiveAdjust -= 10;
                local_X_liveAdjust -= 10;
            }

            else if ((Key)context.EventArgs.GetValue("Key") == Key.W)
            {
                X_LiveAdjust += 10;
                local_X_liveAdjust += 10;
            }

            else if ((Key)context.EventArgs.GetValue("Key") == Key.D)
            {
                Y_LiveAdjust -= 10;
                local_Y_liveAdjust -= 10;
            }

            else if ((Key)context.EventArgs.GetValue("Key") == Key.A)
            {
                Y_LiveAdjust += 10;
                local_Y_liveAdjust += 10;
            }

            else
            {
                return;
            }

            if (Activated)
            {
                LastSnaped.RotateAroundZ(local_Z_liveAdjust);
                LastSnaped.RotateAroundY(local_Y_liveAdjust);
                LastSnaped.RotateAroundX(local_X_liveAdjust);
                showToolOnTarget(_TeachContext.ActiveRobot, LastSnaped);
                _Application.ActiveWindow.Render();
            }
        }

        public void WindowEvent(object sender, EventArgs e)
        {
            /// <summary>Method for catching window events. Used only for returning focus back to Snap window.>

            if (Activated == true)
            {
                IInputElement newFocus = (IInputElement)e.GetValue("NewFocus");
                if (newFocus != null)
                {
                    //print("Interrupted by " + newFocus.ToString());
                }

                if ((bool)sender.GetValue("IsVisible") == false)
                {
                    ToggleSnap(true);
                }

                Keyboard.ClearFocus();
                Keyboard.Focus(InputElement);
                //print("Focus returned");
            }
        }
        
        public void DeactivationEvents(object sender, EventArgs e)
        {
            /// <summary>Method for catching SimulationStarted events. Disable Snap.>

            if (Activated == true)
            {
                ToggleSnap(true);
            }
        }

        public void ContinuousSnap(object sender, EventArgs e)
        {
            /// <summary> Method for catching Snap events. </summary>

            _snapAction.ContinuousResultMouseMove -= ContinuousSnap;
            StatusBar = "Ready";
            SnapResult snapResult = (SnapResult)sender.GetValue("SnapResult");

            Matrix OrigTransformed;
            double snapDist = 0;

            #region All checks before real Snap
            Vector3 snapPos = snapResult.Position;
            if (snapResult.Normal == null)
            {
                StatusBar = "Snap target is missing normal value";
                print("Snap target is missing normal value");
                _snapAction.ContinuousResultMouseMove += ContinuousSnap;
                return;
            }

            Vector3 snapNor = (Vector3)snapResult.Normal;

            //NormalList = snapNor;
            //snapNor = NormalList;

            snapDist = Math.Sqrt(Math.Pow(snapPos.X - LastSnaped.Px, 2) + Math.Pow(snapPos.Y - LastSnaped.Py, 2) + Math.Pow(snapPos.Z - LastSnaped.Pz, 2));

            if ((snapResult.SnapType != SnapType.Face) || (snapResult.VisualObject == null) || (snapDist < 10))
            {
                //StatusBar = "Snap target is not Face";
                //print("Snap target is not Face");
                _snapAction.ContinuousResultMouseMove += ContinuousSnap;
                return;
            }

            ISimComponent comp = (ISimComponent)snapResult.VisualObject.GetValue("Component");
            int indexCategory = comp.PropertyNames.ToList().IndexOf("Category");
            if (!(bool)comp.PropertyValues.ElementAt(indexCategory).ToString().Contains("Parts"))
            {
                StatusBar = "Component is not in 'Parts'";
                _snapAction.ContinuousResultMouseMove += ContinuousSnap;
                return;
            }
            IRobot robot = _TeachContext.ActiveRobot;
            OrigTransformed = transformPosition(robot, snapPos, snapNor);

            if (OrigTransformed == new Matrix())
            {
                print("OrigTransformed is empty !!!");
                _snapAction.ContinuousResultMouseMove += ContinuousSnap;
                return;
            }
            #endregion

            Matrix TransformedTarget = new Matrix();

            TransformedTarget.SetP(OrigTransformed.GetP());
            TransformedTarget.SetN(OrigTransformed.GetN());
            TransformedTarget.SetO(OrigTransformed.GetO());
            //if (lastSnaped != null)
            //{
            //    TransformedTarget.SetN(lastSnaped.GetN());
            //    TransformedTarget.SetO(lastSnaped.GetO());
            //}
            TransformedTarget.SetA(OrigTransformed.GetA());
            
            TransformedTarget.UniformZ();

            TransformedTarget.RotateAroundZ(Z_LiveAdjust);
            TransformedTarget.RotateAroundY(Y_LiveAdjust);
            TransformedTarget.RotateAroundX(X_LiveAdjust);

            LastSnaped = TransformedTarget;
            showToolOnTarget(robot, LastSnaped);
            _Application.ActiveWindow.Render();
            _snapAction.ContinuousResultMouseMove += ContinuousSnap;
            return;
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged([CallerMemberName] string name = null)
        {
            if (ManualSnap)
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
            }
        }

        #endregion

        #region Getters
        double getDistanceTarget_TCP(Matrix TCPmtx, Matrix Targetmtx)
        {
            return Math.Sqrt(Math.Pow(TCPmtx.Px - Targetmtx.Px, 2) + Math.Pow(TCPmtx.Py - Targetmtx.Py, 2) + Math.Pow(TCPmtx.Pz - Targetmtx.Pz, 2));
        }

        (double visibility, int lockedFase, int visibleLEDs) getTrakerData()
        {
            //TrackNow.StartBatch();
            TrackNow.Value = !(bool)TrackNow.Value;
            //TrackNow.EndBatch(false);

            ForcedInterrupt = Keyboard.IsKeyDown(Key.Escape);

            int visibleLeds = (int)TargetLed.Value;
            int lockedFace = (int)TargetSide.Value;

            //double refVisibility = (double)I__AngleSignal.Value;

            if (visibleLeds < 6 ) 
            { 
                return (double.MaxValue, 0, 0); 
            }

            //double ledCoeficient = 7 / visibleLeds;
            //double visibility = refVisibility * ledCoeficient;

            //return (visibility, lockedFace, visibleLeds);

            return ((double)I__AngleSignal.Value * 7 / visibleLeds, lockedFace, visibleLeds);
        }

        Matrix getTCP_Helper()
        {
            if (Current_TSGeometry != null)
            {
                foreach (IFeature feature in Current_TSGeometry.Features)
                {
                    if (feature.Name.Contains("TCP_helper"))
                    {

                        return feature.TransformationInReference.Inverse();
                    }
                }
            }
            return new Matrix();
        }

        public ISimComponent getTSGeometry(ISimNode inputNode)//IRobot robot)
        {
            /// <summary> Get T-Scan_Geometry </summary>
            /// <param name="robot"> </param>
            /// <returns> ISimComponent T-Scan_Geometry </returns>

            ISimComponent nodeTSGeometry = null;
            //ISimComponent tempComp = robot.Component;
            IReadOnlyCollection<ISimNode> children = inputNode.Children;// (IReadOnlyCollection<ISimNode>)tempComp.GetValue("Children");
            //int tryCount = 0;
            //while ((nodeTSGeometry == null) && (tryCount < 100))
            //{
            foreach (ISimNode child in children)
            {
                if (child.Features.Count > 0)
                {
                    foreach (IFeature feature in child.Features)
                    {
                        if (feature.Name.Contains("TCP_helper"))
                        {
                            nodeTSGeometry = child.Component;
                            break;
                        }
                    }
                }
                if (nodeTSGeometry != null) { break; }
                if (child.Children.Count > 0)
                {
                    nodeTSGeometry = getTSGeometry(child);
                }
                if (nodeTSGeometry != null) { break; }
                //var temp_children = (IReadOnlyCollection<ISimNode>)child.GetValue("Children");
                //if (temp_children.Count != 0) 
                //{ 
                //    children = temp_children; 
                //}
            }
                //tryCount++;
            //}

            if (nodeTSGeometry == null)
            {
                //print("Missing TCP_helper frame in T-Scan geometry");
                return null;
            }

            if (nodeTSGeometry.Name.Contains("Mac"))
            {
                ToolTmac = true;
            }
            else
            {
                ToolTmac = false;
            }

            return nodeTSGeometry;
        }

        public ISimComponent getCurrentTracker(IRobot robot)
        {
            /// <summary> Method is searching for trackar, which is sending the same value as inputed robots T-Scan is receiving. </summary>
            /// <param name="robot"> </param>
            /// <returns> Current visibility value. </returns>
            /// 
            IRobotController robotController = robotController = robot.RobotController;
            if (robotController.CurrentTool == null)
            {
                StatusBar = "Please choose tool: T-Scan";
                return null;
            }
            IRobotFrame CurrTool = robotController.CurrentTool;
            ISimNode toolParent = CurrTool.Node.Parent;
            if (!toolParent.Name.Contains("T-Scan") && !toolParent.Name.Contains("T-Mac"))
            {
                //StatusBar = "Please choose tool: T-Scan";
                return null;
            }
            ISimComponent thisScanner = toolParent.Component;
            double TrackerSignalVis = 0;
            var signal = thisScanner.FindBehavior("I__AngleSignal");
            if (signal == null)
            {
                //StatusBar = "T-Scan component is missing I__AngleSignal";
                return null;
            }
            double ScannerSignalVis = (double)thisScanner.FindBehavior("I__AngleSignal").GetValue("Value");

            foreach (ISimComponent tracker in _Application.World.Components)
            {
                var TrackerSignal = tracker.FindBehavior("I__AngleSignal");
                if (TrackerSignal == null)
                {
                    continue;
                }
                TrackerSignalVis = (double)tracker.FindBehavior("I__AngleSignal").GetValue("Value");
                if (ScannerSignalVis == TrackerSignalVis && thisScanner.Name != tracker.Name)
                {
                    return tracker;
                }
            }

            return null;
        }

        #endregion

        #region Helpers

        public double angle_triangle(Matrix pos1, Matrix pos2, Matrix pos3)
        {
            double x1 = pos1.Px;
            double y1 = pos1.Py;
            double z1 = pos1.Pz;

            double x2 = pos2.Px;
            double y2 = pos2.Py;
            double z2 = pos2.Pz;

            double x3 = pos3.Px;
            double y3 = pos3.Py;
            double z3 = pos3.Pz;

            // 1. Joint 5
            // 2. Joint 3
            // 3. Joint 6

            double num = (x2 - x1) * (x3 - x1) +
                      (y2 - y1) * (y3 - y1) +
                      (z2 - z1) * (z3 - z1);

            double den = Math.Sqrt(Math.Pow((x2 - x1), 2) +
                                   Math.Pow((y2 - y1), 2) +
                                   Math.Pow((z2 - z1), 2)) *
                         Math.Sqrt(Math.Pow((x3 - x1), 2) +
                                   Math.Pow((y3 - y1), 2) +
                                   Math.Pow((z3 - z1), 2));

            double angle = Math.Acos(num / den) *
                           (180.0 / 3.141592653589793238463);

            if (double.IsNaN(angle))
            {
                //return 0;
            }
            return angle;
        }

        void print(string toprint)
        {
            _MessageService.AppendMessage(toprint, MessageLevel.Warning);
            //_Application.WriteLine(toprint);
        }

        #endregion

        private string statusBar;
        public string StatusBar
        {
            get { return statusBar; }
            set
            {
                statusBar = value;
                OnPropertyChanged();
            }
        }

        
    }

}
