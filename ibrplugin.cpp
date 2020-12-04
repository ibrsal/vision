#include "SamplePlugin.hpp"

// Additional buttons added, otherwise identical to sample
SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);


	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    //ibr connect(_sparseStereo, SIGNAL(pressed()), this, SLOT(btnPressed()) );
   //ibr connect(_sparseStereoTest, SIGNAL(pressed()), this, SLOT(btnPressed()) );

	_framegrabber = NULL;
	
	_cameras = {"Camera_Right", "Camera_Left"};
	_cameras25D = {"Scanner25D"};
}

// code from sampleplugin
SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

// code from sampleplugin
void SamplePlugin::initialize()
{
	log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

	// Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Desktop/Project_WorkCell/Scene.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

}

// targetFrame added, otherwise identical to sample
void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL)
    {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame(_cameras[0]);
        if (cameraFrame != NULL)
        {
            if (cameraFrame->getPropertyMap().has("Camera"))
            {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new GLFrameGrabber(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }

        Frame* cameraFrame25D = _wc->findFrame(_cameras25D[0]);
        if (cameraFrame25D != NULL)
        {

            if (cameraFrame25D->getPropertyMap().has("Scanner25D"))
            {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D = new GLFrameGrabber25D(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber25D->init(gldrawer);
            }
        }

        _device = _wc->findDevice("UR-6-85-5-A");
        targetFrame = _wc->findFrame<rw::kinematics::MovableFrame>("Ball");
        _step = -1;
    }
}

// code from sampleplugin
void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

// code from sampleplugin
Mat SamplePlugin::toOpenCVImage(const Image& img)
{
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

// New buttons added, otherwise identical to sample
void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj==_btn0){
//		log().info() << "Button 0\n";
//		// Toggle the timer on and off
//		if (!_timer25D->isActive())
//		    _timer25D->start(100); // run 10 Hz
//		else
//			_timer25D->stop();
        _timer->stop();
        rw::math::Math::seed();
        double extend = 0.05;
        double maxTime = 60;
        Q from(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        Q to(6, 1.847, -2.465, -1.602, -0.647, 1.571, 0); //From pose estimation
        createPathRRTConnect(from, to, extend, maxTime);


	} else if(obj==_btn1){
        log().info() << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;

	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
	else if( obj==_btn_im ){
		getImage();
	}
	else if( obj==_btn_scan ){
		get25DImage();
	}
    /*ibr else if( obj==_sparseStereo)
    {
        // Get new images from cameras
        getImage();

        // Get projection matrices
        vector<Eigen::Matrix<double, 3, 4>> eigenProjectionMatrices = determineProjectionMatrices();
        Mat camera1ProjectionMatrix, camera2ProjectionMatrix;

        // Convert Eigen::Matrix to Cv::Mat
        eigen2cv(eigenProjectionMatrices[0], camera1ProjectionMatrix);
        eigen2cv(eigenProjectionMatrices[1], camera2ProjectionMatrix);

	    sparseStereo(camera2ProjectionMatrix, camera1ProjectionMatrix);
    }
    else if(obj==_sparseStereoTest)
    {
        sparseStereoTest();
    } */
}

// code from sampleplugin
void SamplePlugin::get25DImage() {
	if (_framegrabber25D != NULL) {
		for( size_t i = 0; i < _cameras25D.size(); i ++)
		{
			// Get the image as a RW image
			Frame* cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
			_framegrabber25D->grab(cameraFrame25D, _state);

			//const Image& image = _framegrabber->getImage();

			const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

			std::ofstream output(_cameras25D[i] + ".pcd");
			output << "# .PCD v.5 - Point Cloud Data file format\n";
			output << "FIELDS x y z\n";
			output << "SIZE 4 4 4\n";
			output << "TYPE F F F\n";
			output << "WIDTH " << img->getWidth() << "\n";
			output << "HEIGHT " << img->getHeight() << "\n";
			output << "POINTS " << img->getData().size() << "\n";
			output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData())
			{
				rw::math::Vector3D<float> p = p_tmp;
				output << p(0) << " " << p(1) << " " << p(2) << "\n";
			}
			output.close();

		}
	}
}

// Return type has been modified, otherwise identical to sample
tuple<Eigen::Matrix<double, 3, 4>, Eigen::Matrix<double, 4, 4>> SamplePlugin::getProjectionMatrix(string frameName) {
    Frame* cameraFrame = _wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            string camParam = cameraFrame->getPropertyMap().get<string>("Camera");
            istringstream iss (camParam, istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0,
                  0, fovy_pixel, height / 2.0, 0,
                  0, 0, 1, 0;

            cout << "Intrinsic parameters:" << endl;
            cout << KA << endl;


            Transform3D<> camPosOGL = cameraFrame->wTf(_state);
            Transform3D<> openGLToVis = Transform3D<>(RPY<>(-Pi, 0, Pi).toRotation3D());
            Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            cout << "Extrinsic parameters:" << endl;
            cout << H.e() << endl;

            return make_tuple(KA, H.e());
        }
    }
    return make_tuple(Eigen::Matrix<double, 3, 4>(), Eigen::Matrix<double, 4, 4>());
}

// Modified from sample to add option of noise and saving copy with prefix
void SamplePlugin::getImage(int noiseMean, int noiseStdDevs, String copyPrefix)
{
	if (_framegrabber != NULL)
	{
		for( size_t i = 0; i < _cameras.size(); i ++)
		{
			// Get the image as a RW image
			Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
			_framegrabber->grab(cameraFrame, _state);

			const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

			// Convert to OpenCV matrix.
			cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

			// Convert to OpenCV image
			Mat imflip, imflip_mat;
			cv::flip(image, imflip, 1);
			cv::cvtColor( imflip, imflip_mat, COLOR_RGB2BGR );

			if(noiseMean != 0 || noiseStdDevs != 0)
            {
			    imflip_mat = addNoise(imflip_mat, noiseMean, noiseStdDevs);
            }

			cv::imwrite(_cameras[i] + ".png", imflip_mat );
			if(copyPrefix != "")
            {
			    cv::imwrite(copyPrefix + " " +_cameras[i] + ".png", imflip_mat);
            }

			// Show in QLabel
			QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
			QPixmap p = QPixmap::fromImage(img);
			unsigned int maxW = 480;
			unsigned int maxH = 640;
			_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		}
	}
}

// code from sampleplugin
void SamplePlugin::timer()
{
    if(0 <= _step && _step < (int)_path.size())
    {
        _device->setQ(_path.at(_step),_state);
        getRobWorkStudio()->setState(_state);
        _step++;
    }
}

// code from sampleplugin
void SamplePlugin::stateChangedListener(const State& state)
{
  _state = state;
}

// code from sampleplugin
bool SamplePlugin::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom)
    {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
        {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}

// code from sampleplugin
void SamplePlugin::createPathRRTConnect(Q from, Q to,  double extend, double maxTime)
{
    _device->setQ(from,_state);
    getRobWorkStudio()->setState(_state);
    CollisionDetector detector(_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,_device,_state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear();
    if (!checkCollisions(_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(_device, _state, detector, to))
        cout << to << " is in colission!" << endl;;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,_path,maxTime);
    t.pause();


    if (t.getTime() >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

	const int duration = 10;

    if(_path.size() == 2){  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for(int i = 0; i < duration+1; i++){
            tempQ.push_back(linInt.x(i));
        }

        _path=tempQ;
    }
}

//  method 1

// Makes a projection matrix from intrinsic and extrinsic parameters - Lecture 2 slides
vector<Eigen::Matrix<double, 3, 4>> SamplePlugin::determineProjectionMatrices()
{
    vector<Eigen::Matrix<double, 3, 4>> projectionMatrices;

    if (_framegrabber != NULL)
    {
        for (size_t i = 0; i < _cameras.size(); i++)
        {
            tuple<Eigen::Matrix<double, 3, 4>, Eigen::Matrix<double, 4, 4>> matrices = getProjectionMatrix(_cameras[i]);

            Eigen::Matrix<double, 3, 4> projectionMatrix = get<0>(matrices) * get<1>(matrices);
            cout << _cameras[i] << " Projection Matrix:" << endl << projectionMatrix << endl;
            projectionMatrices.push_back(projectionMatrix);
        }
    }

    return projectionMatrices;
}

// Based on OpenCV documentation HoughCircles https://docs.opencv.org/3.2.0/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
// medianBlur https://docs.opencv.org/3.2.0/d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9
vector<Vec3f> findHoughCircles(Mat image)
{
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    medianBlur(gray, gray, 5);

    vector<Vec3f> circles;

    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 20, 50, 30, 10, 25);

    std::vector<Vec3f> outputCircles;

    for(size_t i = 0; i < circles.size(); i++)
    {
        // Only look in the bottom 60% of the image
        if(circles[i][1] > image.rows * 0.4)
        {
            outputCircles.push_back(circles[i]);
        }
        else
        {
            cout << "Rejected " << circles[i] << endl;
        }

    }

    return outputCircles;
}

// Based on opencv documentation triangulatePoints https://docs.opencv.org/3.2.0/d9/d0c/group__calib3d.html#gad3fc9a0c82b08df034234979960b778c
Mat triangulate(Mat c1ProjectionMatrix, Mat c2ProjectionMatrix, std::vector<Point2f> leftPoints, std::vector<Point2f> rightPoints)
{
    Mat results;

    triangulatePoints(c1ProjectionMatrix, c2ProjectionMatrix, leftPoints, rightPoints, results);

    return results;
}

// Simple conversion method
vector<Point2f> circlesToPoints(vector<Vec3f> input)
{
    vector<Point2f> output;

    for(size_t i = 0; i < input.size(); i++)
    {
        output.push_back(Point2f(input[i][0], input[i][1]));
    }

    return output;
}

// Conversion method - normalises values from triangulatePoints
Point3f to3DPoint(Mat input)
{
    float x, y, z, val;
    x = input.at<float>(0,0);
    y = input.at<float>(0,1);
    z = input.at<float>(0,2);
    val = input.at<float>(0,3);

    Point3f point = Point3f(x / val, y / val, z / val);
    return point;
}

// Simple conversion method
Vector3D<> toVector3D(Point3f point)
{
    Vector3D<> vecPose(point.x, point.y, point.z);
    return vecPose;
}

// Based on opencv documentation randn https://docs.opencv.org/3.2.0/d2/de8/group__core__array.html#gaeff1f61e972d133a04ce3a5f81cf6808
Mat SamplePlugin::addNoise(Mat image, int mean, int stddevs)
{
    Mat noise(image.size(), image.type());
    randn(noise, Scalar::all(mean), Scalar::all(stddevs));
    image += noise;
    return image;
}

// Runs tests on sparse stereo implementation
/* ibr void SamplePlugin::sparseStereoTest()
{
    // Get projection matrices
    vector<Eigen::Matrix<double, 3, 4>> eigenProjectionMatrices = determineProjectionMatrices();
    Mat camera1ProjectionMatrix, camera2ProjectionMatrix;

    // Convert Eigen::Matrix to Cv::Mat - https://docs.opencv.org/3.2.0/d0/daf/group__core__eigen.html#ga224be3c968ba50c038629355004f0a8b
    eigen2cv(eigenProjectionMatrices[0], camera1ProjectionMatrix);
    eigen2cv(eigenProjectionMatrices[1], camera2ProjectionMatrix);

    if(nullptr==targetFrame)
    {
        return;
    }
    vector<Vec3i> testResults;

    // Iterate through noise levels
    for(int noise = 0; noise <= 150; noise += 10)
    {
        cout << "Noise: " << noise << endl;
        int successCounter = 0;
        int attempts = 0;

        // Iterate through positions
        for (double x = -0.35; x < 0.36; x += 0.05)
        {
            for (double y = 0.40; y < 0.51; y += 0.05)
            {
                cout << "Position - x: " << x << " y: " << y << endl;

                // Set target position
                targetFrame->moveTo(
                        rw::math::Transform3D<>(
                                rw::math::Vector3D<>(x, y, 0.15505),
                                rw::math::RPY<>(0, 0, rw::math::Deg2Rad * 90)
                        ), _state);

                getRobWorkStudio()->setState(_state);

                //String prefix = to_string((int) (x * 100)) + " " + to_string((int) (y * 100));
                String prefix = "";

                // Get new images from cameras
                getImage(0, noise, prefix);

                // Determine position
                Vector3D<> position = sparseStereo(camera2ProjectionMatrix, camera1ProjectionMatrix, false);
                cout << position(0) << " " << position(1) << " " << position(2) << endl;

                cout << "Deviation - X: " << x - position(0) << " Y: " << y - position(1) << endl << endl;

                // Increment successes if we don't get a blank position back
                if(position(0) != 0 && position(1) != 0 && position(2) !=0)
                {
                    successCounter++;
                }
                attempts++;
            }
        }

        // Add this set of results
        testResults.push_back(Vec3i(noise, successCounter, attempts));
    }

    // Print results
    cout << "Results:" << endl;
    for(size_t i = 0; i < testResults.size(); i++)
    {
        cout << testResults[i] << endl;
    }
} */

// Performs sparse stereo pose estimation
/* Vector3D<> SamplePlugin::sparseStereo(Mat leftCameraProjectionMatrix, Mat rightCameraProjectionMatrix, bool plotPath)
{
    // Load images
    Mat leftImage = imread("Camera_Left.png");
    Mat rightImage = imread("Camera_Right.png");

    // Find circles using Hough transform
    std::vector<Vec3f> leftCircles = findHoughCircles(leftImage);
    std::vector<Vec3f> rightCircles = findHoughCircles(rightImage);

    // Get the centerpoints of those circles as points
    std::vector<Point2f> leftPoints = circlesToPoints(leftCircles);
    std::vector<Point2f> rightPoints = circlesToPoints(rightCircles);

    // If points were found, and the same number of points were found in each image, we determine the location
    if(leftPoints.size() > 0 && rightPoints.size() > 0 && leftPoints.size() == rightPoints.size())
    {
        // triangulate the points of the circles
        Mat point = triangulate(leftCameraProjectionMatrix, rightCameraProjectionMatrix, leftPoints, rightPoints);

        // Normalise point
        Point3f correctedPoint = to3DPoint(point);

        Vector3D<> pose = toVector3D(correctedPoint);

        if (plotPath)
        {
            RRTConnect(pose);
        }

        return pose;
    }
    else
    {
        cout << "Invalid number of points found" << endl;
        cout << "Left points: " << leftPoints.size() << endl;
        cout << "Right points: " << rightPoints.size() << endl;

        return Vector3D<>();
    }
}*/

// End of method 1

Q SamplePlugin::CToQConverter(Transform3D<> location,SerialDevice::Ptr robot,Frame::Ptr gripperFrame, State state)
{
    JacobianIKSolver jacobianSolver(robot, gripperFrame.get(), state);
    vector<math::Q> possibleSet = jacobianSolver.solve(location, state);
    math::Q QLocation = possibleSet.at(0);
    return QLocation;
}

int SamplePlugin::RRTConnect(Vector3D<> vec)
{
    std::vector<State> states;
    TimedStatePath tStatePath;
    Vector3D<> vecPose = vec;
    cout << vecPose(0) << "  " << vecPose(1) << "  " << vecPose(2) << endl;

    //UR chain setup, taken from some of the exercises.
    Transform3D<>forwardKinematics(const vector<Transform3D<>>& trefs, const unsigned int x , const Q&q);

    Vector3D<> V0(0, 0, 0);
    RPY<> R0(0, 0, 0);
    Transform3D<> T0(V0, R0.toRotation3D());
    //joint1
    Vector3D<> V1(0, 0, 0.08920);
    RPY<> R1(0, 0, 90 * Deg2Rad);
    Transform3D<> T1(V1, R1.toRotation3D());
    //joint2
    Vector3D<> V2(-0.425, 0, 0);
    RPY<> R2(0, 0, 0);
    Transform3D<> T2(V2, R2.toRotation3D());
    //joint3
    Vector3D<> V3(-0.39243, 0, 0);
    RPY<> R3(0, 0, 0);
    Transform3D<> T3(V3, R3.toRotation3D());
    //joint4
    Vector3D<> V4(0, 0, 0.109);
    RPY<> R4(0, 0, 90 * Deg2Rad);
    Transform3D<> T4(V4, R4.toRotation3D());
    //joint5
    Vector3D<> V5(0, 0, 0.093);
    RPY<> R5(0, 0, -90 * Deg2Rad);
    Transform3D<> T5(V5, R5.toRotation3D());

    vector<Transform3D<>> ur_kinematic_chain = { T0, T1, T2, T3, T4, T5 };
    Q qDefault = Q(6, 1.2, -1.75, -1.6, -1.35, 1.65, 0);

    WorkCell::Ptr workCell = loaders::WorkCellLoader::Factory::load("/home/student/Documents/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml");
    if(workCell==NULL)
    {
        cout << "Scene could not be loaded" << endl;
        return -1;
    }
    // find relevant frames
    SerialDevice::Ptr robotUR5 = workCell->findDevice<SerialDevice>("UR-6-85-5-A");
    if(NULL==robotUR5)
    {
        cout << "Device could not be loaded" << endl;
        return -1;
    }
    MovableFrame::Ptr robotUR5Frame = workCell->findFrame<MovableFrame>("URReference");
    if(NULL==robotUR5)
    {
        cout << "Device could not be loaded" << endl;
        return -1;
    }
    Frame::Ptr tableFrame = workCell->findFrame<Frame>("Table");
    if (NULL == tableFrame)
    {
        RW_THROW("Table could not be loaded");
        return -1;
    }
    MovableFrame::Ptr targetFrame = workCell->findFrame<MovableFrame>("Ball");
    if(NULL==targetFrame)
    {
        cout << "Target could not be loaded" << endl;
        return -1;
    }

    Frame::Ptr gripperFrame = workCell->findFrame<Frame>("WSG50.TCP");
    if (NULL == gripperFrame)
    {
        RW_THROW("TCP could not be loaded");
        return -1;
    }

    State state = workCell->getDefaultState();
    Transform3D<> itemLocation = robotUR5Frame->fTf(targetFrame.get(), state);
    Transform3D<> targetLocation(Vector3D<>(0.48, -0.36, itemLocation.P()(2)), RPY<double>(0* Deg2Rad, 0* Deg2Rad, 90* Deg2Rad));

    const CollisionStrategy::Ptr cdstrategy =
            ProximityStrategyFactory::makeCollisionStrategy("PQP");
    if (cdstrategy.isNull())
        RW_THROW("PQP Collision Strategy could not be found.");
    const CollisionDetector::Ptr collisionDetector = ownedPtr(new CollisionDetector(workCell, cdstrategy));
    const PlannerConstraint con =  PlannerConstraint::make(collisionDetector, robotUR5, state);
    const QToQPlanner::Ptr planner =  RRTPlanner::makeQToQPlanner(con, robotUR5, RRTPlanner::RRTConnect);

    //qDefault;
    Q itemQLoc = CToQConverter( itemLocation, robotUR5, gripperFrame, state);
    QPath toObject;
    if (planner->query(qDefault, itemQLoc, toObject))
    {
        cout << "Planned path with " << toObject.size();
        cout << " configurations" << endl;
    }
    // Map the configurations to a sequence of states.
    const std::vector<State> toObjectStates = Models::getStatePath(*robotUR5, toObject, state);
    robotUR5->setQ(itemQLoc, state);
    targetFrame->setTransform(gripperFrame->fTf(targetFrame.get(), state), state);
    targetFrame->attachTo(gripperFrame.get(), state);

    Q targetQLoc = CToQConverter( targetLocation, robotUR5, gripperFrame, state);

    const PlannerConstraint conCol =  PlannerConstraint::make(collisionDetector, robotUR5, state);
    const QToQPlanner::Ptr plannerCol =  RRTPlanner::makeQToQPlanner(conCol, robotUR5, RRTPlanner::RRTConnect);

    QPath moveObject;
    if (plannerCol->query(itemQLoc, targetQLoc, moveObject))
    {
        cout << "Planned path with " << moveObject.size();
        cout << " configurations" << endl;
    }
    const std::vector<State> moveObjectStates = Models::getStatePath(*robotUR5, moveObject, state);
    robotUR5->setQ(targetQLoc, state);
    targetFrame->setTransform(tableFrame->fTf(targetFrame.get(), state), state);
    targetFrame->attachTo(tableFrame.get(), state);
    const PlannerConstraint conPlac =  PlannerConstraint::make(collisionDetector, robotUR5, state);
    const QToQPlanner::Ptr plannerPlac =  RRTPlanner::makeQToQPlanner(conPlac, robotUR5, RRTPlanner::RRTConnect);

    QPath returnToDefault;
    if (planner->query(targetQLoc, qDefault, returnToDefault))
    {
        cout << "Planned path with " << returnToDefault.size();
        cout << " configurations" << endl;
    }
    const std::vector<State> returnToDefaultStates = Models::getStatePath(*robotUR5, returnToDefault, state);

    QPath path;
    // Map the configurations to a sequence of states.
    for (size_t j = 0; j < toObjectStates.size() ; ++j)
    {
        states.push_back(toObjectStates.at(j));
    }
    for (size_t j = 0; j < moveObjectStates.size() ; ++j)
    {
        states.push_back(moveObjectStates.at(j));
    }
    for (size_t j = 0; j < returnToDefaultStates.size() ; ++j)
    {
        states.push_back(returnToDefaultStates.at(j));
    }


    // Write the sequence of states to a file.
    PathLoader::storeVelocityTimedStatePath(
            *workCell, states, "/home/student/Documents/Project_WorkCell_Cam/Project_WorkCell/QTQPathVel.rwplay");

    return 0;
}
