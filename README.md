## Guidelines
1. This project requires argos3 (https://github.com/ilpincy/argos3) to be installed.
	The version to work with is:
	```bash
	commit c04be869311801976a83613552e111b2eef4dd45 (HEAD -> master, origin/master, origin/HEAD)
	Author: Michael Allwright <allsey87@gmail.com>
	Date:   Wed Dec 8 13:44:48 2021 +0100

	Fix the left-right wheel offset in the 3D dynamics model for the Pi-Puck (#196)
	```
	Clone, checkout the right version, and create a branch for it:
	```bash
	git clone https://github.com/ilpincy/argos3
	cd argos3
	git checkout c04be869311801976a83613552e111b2eef4dd45
	git switch -c ros
	```

2. Before compiling and installing argos, it is highly recommended to delete old version of argos installation from your system. To do that, check /usr/local, which is the default place where argos installs itself. To check:
	```bash
	cd /usr/local
	find . -name "*argos*"
	```

	All the files that contains the name argos will be listed. **Check carefully** to delete only argos files but not other system files. Usually, something like:
	```bash
	rm -rf */argos3
	```
	will do, but again, **check carefully**.

	After removing old version of argos installation, and before compiling our version, you may want to apply some patches for argos3 from folder `argos3-patch`, depends on what do you need. For mns3.0 experiments, the patches that are directly under `argos-patch` are essential. Some patches in `backup` folder might also be useful for some debug purposes.

	To apply all the essential patches, you can simply :
	```bash
	cd argos3
	git apply ../argos_ros_simulator/argos-patch/*.patch
	git commit -m "Apply the patches for mns3.0"
	```
	or apply them one by one
	```bash
	cd argos3
	git apply ../argos_ros_simulator/argos-patch/xxx.patch
	git commit -m "Apply the patch for xxx"
	```