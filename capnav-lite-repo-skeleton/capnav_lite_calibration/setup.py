from setuptools import setup

package_name = "capnav_lite_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CapNav-Lite contributors",
    maintainer_email="maintainers@example.org",
    description="CapNav-Lite calibration tool (skeleton).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calibrate_profile = capnav_lite_calibration.calibrate_profile:main",
        ],
    },
)
