#!/usr/bin/env python

import sys
from os import path, walk
from shutil import rmtree
from pathlib import Path
from setuptools import setup
from setuptools.command.build_py import build_py

def print_error(*args, **kwargs):
    """ Print in stderr. """
    print(*args, file=sys.stderr, **kwargs)


def find_resources(package_name):
    """ Find the relative path of files under the resource folder. """
    resources = []
    package_dir = path.join("src", package_name)
    resources_dir = path.join(package_dir, package_name)

    for (root, _, files) in walk(resources_dir):
        for afile in files:
            if (afile != package_name and 
                not afile.endswith(".DS_Store") and
                not afile.endswith(".py")):
                rel_dir = path.relpath(root, package_dir)
                src = path.join(rel_dir, afile)
                resources.append(src)
    return resources

# Package name.
package_name = "robot_properties_talos"
package_version = "1.0.0"

# Find the resource files.
resources = find_resources(package_name)

# Install nodes and demos.
scripts_list = []
for (root, _, files) in walk(path.join("demos")):
    for demo_file in files:
        scripts_list.append(path.join(root, demo_file))

class custom_build_py(build_py):
    def _build_doc(self):
        """Build the documentation if the mpi_cmake_module is installed."""
        # Try to build the doc and install it.
        try:
            # Get the mpi_cmake_module build doc method
            from mpi_cmake_modules.documentation_builder import (
                build_documentation,
            )

            build_documentation(
                str((Path(self.build_lib) / package_name / "doc").absolute()),
                str(Path(__file__).parent.absolute()),
                package_version,
            )
        except ImportError as e:
            print_error()

    def _build_xacro(self):
        """ Look for the xacro files and build them in the build folder. """
        resources_dir = str(
            Path(__file__).parent.absolute()
            / "src"
            / package_name
            / "resources"
        )
        build_folder = str(
            (
                Path(self.build_lib) / package_name / "resources" / "urdf"
            ).absolute()
        )
        xacro_files = []
        for (root, _, files) in walk(str(Path(resources_dir) / "xacro")):
            for afile in files:
                if afile.endswith(".urdf.xacro"):
                    xacro_files.append(str(Path(root) / afile))

        # rebuild all urdfs.
        rmtree(build_folder, ignore_errors=True)
        Path(build_folder).mkdir(parents=True, exist_ok=True)

        for xacro_file in xacro_files:
            for xacro_file in xacro_files:
                # Generated file name
                generated_urdf_path = str(
                    Path(build_folder) / Path(xacro_file).stem
                )
                self._build_single_xacro_file(xacro_file, generated_urdf_path)

    def _build_single_xacro_file(self, input_path, output_path):
        from xacro import process_file, open_output
        from xacro.xmlutils import xml

        unicode = str
        encoding = {}
        print_error(
            "building xacro file (", input_path, ") into (", output_path, ")"
        )
        try:
            # open and process file
            doc = process_file(input_path)
            # open the output file
            out = open_output(output_path)

        except xml.parsers.expat.ExpatError as e:
            print_error("XML parsing error: %s" % unicode(e), alt_text=None)

        except Exception as e:
            msg = unicode(e)
            if not msg:
                msg = repr(e)
            print_error(msg)

        # write output
        out.write(doc.toprettyxml(indent="  ", **encoding))
        # only close output file, but not stdout
        out.close()

    def run(self):
        """Build the package. """
        # build documentation.
        # self._build_doc()
        # build the xacro files into urdf files.
        # self._build_xacro()
        # distutils uses old-style classes, so no super()
        build_py.run(self)

# Final setup.
setup(
    name=package_name,
    version="1.0.0",
    package_dir={package_name: path.join("src", package_name)},
    packages=[package_name],
    package_data={package_name: resources},
    scripts=scripts_list,
    install_requires=["setuptools", 
                    #   "xacro", 
                      "pybullet", 
                      "importlib_resources",
                      "meshcat",
                      "bullet_utils"],
    zip_safe=True,
    maintainer="skleff",
    maintainer_email="sk8001@nyu.edu",
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    description="Wrapper around the pybullet interface using pinocchio.",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    cmdclass={"build_py": custom_build_py},
)
