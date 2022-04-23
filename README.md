# ROS_Path_Planning

## Workspace

Workspace structure

```bash
ROS_Path_Planning/ # Workspace Root
	build/
	install/
	log/
	src/ # All packages in src
		interfaces/ # custom interfaces (c++)
		path_planning/ # path planner (python)
			path_planning/ # Code of package
				__init__.py
				path_planner.py
                		...
			resource/ # maps, ...
			test/
			package.xml
			setup.cfg
			setup.py
```

## Package

Package structure

- `package.xml` file containing meta information about the package
- `setup.py` containing instructions for how to install the package
- `setup.cfg` is required when a package has executables, so `ros2 run` can find them
- `/<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py`

## Get Started

1. (Optional) Edit config files

   `package.xml`: name, version, description, maintainer + email, license, dependencies

   `setup.py`: name, version, maintainer, email, description, license, entry points

   `setup.cfg`: should already be ok
   
2. Initialize submodules (fszhaw_msgs)

   `git submodule update --init`

3. Install dependencies (from workspace root `ROS_Path_Planning`)

   `rosdep install -i --from-path src --rosdistro foxy -y`

4. Build packages (from workspace root `ROS_Path_Planning`)

   Optional: If colcon command is still missing

   `sudo apt install python3-colcon-common-extensions`

   `colcon build`

   or

   `colcon build --packages-select <package_name>`

5. Source setup files (in a new shell)

   `. install/setup.bash`

6. Run package

   `ros2 run <package_name> <entry_point>`

   e.g.

   `ros2 run path_planning path_planner`
