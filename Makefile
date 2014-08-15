all:
	python build_includeActors.py
	catkin_make
clean:
	rm -rf devel build
	
