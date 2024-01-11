from setuptools import find_packages, setup

package_name = "my_robot_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="28587497",
    maintainer_email="49950899+Olseda20@users.noreply.github.com",
    description="Limo robot controller for the Robot Programming assignment",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # "executable_name = package_name.filename:method_to_run"
            "simple_pothole_detector = my_robot_controller.simple_pothole_detector:main",
            "pothole_mapper = my_robot_controller.pothole_mapper:main",
            "pothole_reporter = my_robot_controller.pothole_reporter:main",
        ],
    },
)
