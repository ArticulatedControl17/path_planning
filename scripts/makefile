all:
	g++ -c -std=gnu++11 -fPIC error_calc.cpp
	g++ -c -std=gnu++11 -fPIC helper_functions.cpp
	g++ -c -std=gnu++11 -fPIC model.cpp
	g++ -c -std=gnu++11 -fPIC pathPlanning.cpp
	g++ -c -std=gnu++11 -fPIC Point.cpp
	g++ -c -std=gnu++11 -fPIC pp_wrapper.cpp
	g++ -c -std=gnu++11 -fPIC recalculatePath.cpp
	g++ -c -std=gnu++11 -fPIC track_checker.cpp
	g++ -c -std=gnu++11 -fPIC vehicleState.cpp
	g++ -c -std=gnu++11 -fPIC vehicleState_error.cpp
	g++ -std=gnu++11 -fPIC pp_wrapper.cpp pathPlanning.cpp Point.cpp track_checker.cpp vehicleState.cpp vehicleState_error.cpp helper_functions.cpp error_calc.cpp model.cpp recalculatePath.cpp
	g++ -shared -Wl,-soname,libpp.so -o libpp.so  pp_wrapper.o pathPlanning.o Point.o track_checker.o vehicleState.o vehicleState_error.o helper_functions.o error_calc.o model.o recalculatePath.o
