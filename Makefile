all:
	catkin_make
	python src/se306_p1_pkg/world/updateactors.py
clean:
	rm -rf devel build
	
