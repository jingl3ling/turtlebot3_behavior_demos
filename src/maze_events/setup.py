from setuptools import setup

package_name = "maze_events"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        # Runtime Python deps (install via pip in your environment):
        # ultralytics, opencv-python, pillow, zenoh, psycopg2-binary
    ],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="HW2 detection + Zenoh + PostgreSQL pipeline for TurtleBot maze.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = maze_events.detector_node:main",
            "zenoh_ingest_worker = maze_events.zenoh_ingest_worker:main",
        ],
    },
)

