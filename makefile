
YouFlowers : src/hmain.o src/Tracker.o src/TwoCamSolver.o src/QuickTracker.o src/cap_libv4l_cust.o
	g++ -o YouFlowers src/hmain.o src/cap_libv4l_cust.o src/Tracker.o src/TwoCamSolver.o src/QuickTracker.o \
		-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lboost_program_options-mt

src/hmain.o : src/hmain.cpp include/lop.h include/solve.h include/Timings.h include/TrackedGroup.h include/TrackedPoint.h include/Tracker.h include/TrackParams.h include/FlowerDef.h include/FlowerMove.h include/ardServo.h
	g++ -DMAC -I/opt/local/include -I/opt/local/include/opencv -Iinclude -O0 -g3 -Wall -c -fmessage-length=0 -o src/hmain.o src/hmain.cpp

src/Tracker.o : src/Tracker.cpp include/lop.h include/solve.h include/Timings.h include/TrackedGroup.h include/TrackedPoint.h include/Tracker.h include/TrackParams.h
	g++ -DMAC  -I/opt/local/include -I/opt/local/include/opencv -Iinclude -O0 -g3 -Wall -c -fmessage-length=0 -o src/Tracker.o src/Tracker.cpp

src/TwoCamSolver.o : src/TwoCamSolver.cpp include/TwoCamSolver.h include/lop.h include/solve.h include/Timings.h include/TrackedGroup.h include/TrackedPoint.h include/Tracker.h include/TrackParams.h
	g++ -DMAC  -I/opt/local/include -I/opt/local/include/opencv -Iinclude -O0 -g3 -Wall -c -fmessage-length=0 -o src/TwoCamSolver.o src/TwoCamSolver.cpp

src/QuickTracker.o : src/QuickTracker.cpp include/QuickTracker.h include/lop.h include/solve.h include/Timings.h include/TrackedGroup.h include/TrackedPoint.h include/Tracker.h include/TrackParams.h \
	include/FindAreas.h include/RectZone.hpp
	g++ -DMAC  -I/opt/local/include -I/opt/local/include/opencv -Iinclude -O0 -g3 -Wall -c -fmessage-length=0 -o src/QuickTracker.o src/QuickTracker.cpp

src/cap_libv4l_cust.o : src/cap_libv4l_cust.cpp
	g++ -DMAC  -I/opt/local/include -I/opt/local/include/opencv -Iinclude -I/home/ubuntu/OpenCV-2.4.1/modules/highgui/src -I/home/ubuntu/OpenCV-2.4.1/build -I/usr/include/eigen2 \
		-O0 -g3 -Wall -c -fmessage-length=0 -o src/cap_libv4l_cust.o src/cap_libv4l_cust.cpp
	