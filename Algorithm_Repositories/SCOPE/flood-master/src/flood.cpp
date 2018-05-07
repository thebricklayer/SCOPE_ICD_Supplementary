#include <fstream>
#include <cstdlib>
#include <ctime>
#include "flood.h"
#if RENDER
#include "render.h"
#endif

FLOOD::FLOOD() {
	// Load model into kd-tree
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	// POSE is unkown at start
	finding = true;
	done = false;
	exit = false;
	// Define quaternion for 45 degree rotation about each axis
	rotx.w = 0.924; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0;   rotz.x = 0.0;
	rotx.y = 0.0;   roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0;   roty.z = 0.0;   rotz.z = 0.383;
	pthread_mutex_init(&lock, NULL);
	pthread_mutex_init(&sa_lock, NULL);
	sem_init(&frame1, 0, 0);
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	// Free model/tree
	freeModel(faces);
	deleteTree(root,0);
	// Release mutex/semaphore rescources
	pthread_mutex_destroy(&lock);
	pthread_mutex_destroy(&sa_lock);
	sem_destroy(&frame1);
};

void FLOOD::run() {
	std::thread frames(&FLOOD::getFrame, this); // Thread to read frames in
	std::thread icp(&FLOOD::calcPose, this); // Thread to calculate POSE
#if RENDER
	std::thread animate(&FLOOD::render, this); // Thread to render results in real time
#endif
	frames.join(); // Cleanup threads
	icp.join();
#if RENDER
		animate.join();
#endif
}

void FLOOD::calcPose() {
	unsigned int i, j;
	quat current, temp;
	// Initialize timer
	clock_t looking, found;
	auto start = std::chrono::system_clock::now();
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	float error, RT[3][3], t[3];
	double tm = 0, acq_time;
	// Read all trajectories being tested
    std::string dir = FRAME_DIRECTORIES, pos, rot;
#if TO_FILE
    // Output results
    pos = "positon_big_3_9.txt";
    rot = "orientation_big_3_9.txt";
    FILE *fpos = std::fopen(pos.c_str(), "w");
    FILE *frot = std::fopen(rot.c_str(), "w");
#endif
	q.w = 1; q.x = 0; q.y = 0; q.z = 0;
	while(!exit) {
		// start timing current iteration
		start = std::chrono::system_clock::now();
		// Wait for new frame
		sem_wait(&frame1);
		pthread_mutex_lock(&lock);
		// Calculate pose using known state
		if(!finding) {
			error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
			if(error > 0.035)
				finding = true;
		} else {  // Calculate pose with unknown state
			float best = 100;
			float Temp[4][4];
			current = q;
			looking = clock();
			point4D initState[numPts];
			// Remember initial state to do multiple trials
			memcpy(initState, scan, numPts*sizeof(point4D));
			initializePose(current, translation, Temp);
			// Trial with no rotation
			error = icp(initState, root, Temp, numPts, MAX_ITERATIONS_FIND);
			// Rotate about x axis and try each result
			for(int j=0; j<7; j++) {
				if(error < best) {
					best = error;
					memcpy(T, Temp, 16*sizeof(float));
				}
				memcpy(initState, scan, numPts*sizeof(point4D));
				temp = multQuat(rotx, current);
				current = temp;
				initializePose(current, translation, Temp);
				error = icp(initState, root, Temp, numPts, MAX_ITERATIONS_FIND);
			}
			found = clock();
			acq_time = (double) (found-looking) / CLOCKS_PER_SEC;
			finding = false;
		}
		// Rotate position vector
		RT[0][0] = T[0][0]; RT[0][1] = T[1][0]; RT[0][2] = T[2][0];
		RT[1][0] = T[0][1]; RT[1][1] = T[1][1]; RT[1][2] = T[2][1];
		RT[2][0] = T[0][2]; RT[2][1] = T[1][2]; RT[2][2] = T[2][2];
		t[0] = T[0][3]; t[1] = T[1][3]; t[2] = T[2][3];
		matMulVec3D(RT, t, translation);
		// Print results
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;
#if TO_FILE
		// Print current translation vector and quaternion
		printf("%f\n", error);
		fprintf(fpos, "%f  ", elapsed_seconds);
		fprintf(frot, "%f  ", elapsed_seconds);
		printTrans(T, translation, fpos, frot);
#else
		printf("%f\n", error);
		printTrans(T, translation, NULL, NULL);
#endif
		// Done using current frame
		pthread_mutex_unlock(&lock);
	}
#if TO_FILE
	std::fclose(fpos);
	std::fclose(frot);
#endif
	printf("Time to acquire: %fs\n", acq_time);
}

void FLOOD::initializePose(quat qInit, float t[4], float Temp[4][4]) {
	quat2trans(Temp,qInit,t);
}

void FLOOD::getFrame() {
	// Start camera camera driver
	o3d3xx::Logging::Init();
	o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
	// Start framegrabber
	o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
	o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
	int fileNum = 1, val = 0;
	// Initialize translation/dimensions to filter frame from LiDAR using bounding box
	float t[3], dims[3] = {0.15, 0.7, 0.7};
	t[0] = -translation[0]; t[1] = -translation[1]; t[2] = -translation[2];
	img->setPosition(t, dims);
	std::vector<point4D> v;
	while(!exit) {
		if (! fg->WaitForFrame(img.get(), 1000)) {
			std::cerr << "Timeout waiting for camera!" << std::endl;
			continue;
		}
		v = img->XYZImage(); // Get frame
		v = hcluster(v); // Run clustering algorithm 
		pthread_mutex_lock(&lock); // Wait for until pose has been calculated using previous frame
		numPts = v.size();
		std::copy(v.begin(), v.end(), scan); // Copy new frame to image buffer
		++fileNum;
		// Update bounding box position
		t[0] = -translation[0]; t[1] = -translation[1]; t[2] = -translation[2];
		img->setPosition(t, dims);
		sem_post(&frame1);
		pthread_mutex_unlock(&lock);
	}
}

void FLOOD::getPosition(float position) {
	// Set initial position, and pass to renderer
	translation[0] = -position; translation[1] = 0; translation[2] = 0; translation[3] = 1;
#ifdef RENDER
	pthread_mutex_lock(&sa_lock);
	shared_array[0] = 1; shared_array[1] = 0;
	shared_array[2] = 0; shared_array[3] = 0;
	shared_array[4] = translation[0];
	shared_array[5] = translation[1];
	shared_array[6] = translation[2];
	done = true;
	pthread_mutex_unlock(&sa_lock);
#endif
}

void FLOOD::printQuat(quat qt, FILE *f) {
	if(f) {
		fprintf(f, "%f  %f  %f  %f\n", qt.w, qt.x, qt.y, qt.z);
	} else {
		printf("%f  %f  %f  %f\n", qt.w, qt.x, qt.y, qt.z);
	}
}

void FLOOD::printTrans(float T[4][4], float translation[3], FILE *pos, FILE *rot) {
	quat qt;
	// Convert translation matrix to quaternion
	trans2quat(T, &qt);
#ifdef RENDER
	// Pass quaternion/position to renderer
	pthread_mutex_lock(&sa_lock);
	shared_array[0] = qt.w; shared_array[1] = qt.x;
	shared_array[2] = qt.y; shared_array[3] = qt.z;
	shared_array[4] = translation[0];
	shared_array[5] = translation[1];
	shared_array[6] = translation[2];
	done = true;
	pthread_mutex_unlock(&sa_lock);
#endif
	printQuat(qt, rot);
	if(pos) {
		fprintf(pos, "%f  %f  %f\n", translation[0], translation[1], translation[2]);
	} else {
		printf("%f  %f  %f\n", translation[0], translation[1], translation[2]);
	}
}

#if RENDER
void FLOOD::render() {
	Render scene(shared_array, &done, &sa_lock, &exit);
	scene.run();
}
#endif