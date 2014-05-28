echo "Removing any pre-installed ffmpeg and x264"
sudo apt-get -qq remove ffmpeg x264 libx264-dev
echo "Installing Dependencies ..."
echo "libopencv-dev"
sudo apt-get -qq install libopencv-dev
echo "build-essential"
sudo apt-get -qq install build-essential
echo "checkinstall"
sudo apt-get -qq install checkinstall
echo "cmake"
sudo apt-get -qq install cmake
echo "pkg-config"
sudo apt-get -qq install pkg-config
echo "yasm"
sudo apt-get -qq install yasm
echo "libjpeg-dev"
sudo apt-get -qq install libjpeg-dev
echo "libjasper-dev"
sudo apt-get -qq install libjasper-dev
echo "libavcodec-dev"
sudo apt-get -qq install libavcodec-dev
echo "libavformat-dev"
sudo apt-get -qq install libavformat-dev
echo "libopencv-dev"
sudo apt-get -qq install libopencv-dev
echo "libswscale-dev"
sudo apt-get -qq install libswscale-dev
echo "libdc1394-22-dev"
sudo apt-get -qq install libdc1394-22-dev
echo "libxine-dev"
sudo apt-get -qq install libxine-dev
echo "libgstreamer0.10-dev"
sudo apt-get -qq install libgstreamer0.10-dev
echo "libgstreamer-plugins-base0.10-dev"
sudo apt-get -qq install libgstreamer-plugins-base0.10-dev
echo "libv41-dev"
sudo apt-get -qq install libv4l-dev
echo "python-dev"
sudo apt-get -qq install python-dev
echo "python-numpy"
sudo apt-get -qq install python-numpy
echo "libtbb-dev"
sudo apt-get -qq install libtbb-dev
echo "libqt4-dev"
sudo apt-get -qq install libqt4-dev
echo "libgtk2.0-dev"
sudo apt-get -qq install libgtk2.0-dev
echo "libfaac-dev"
sudo apt-get -qq install libfaac-dev
echo "libmp31ame-dev"
sudo apt-get -qq install libmp3lame-dev
echo "libopencore-amrnb-dev"
sudo apt-get -qq install libopencore-amrnb-dev
echo "libopencore-amrwb-dev"
sudo apt-get -qq install libopencore-amrwb-dev
echo "libtheora-dev"
sudo apt-get -qq install libtheora-dev
echo "libvorbis-dev"
sudo apt-get -qq install libvorbis-dev
echo "libxvidcore-dev"
sudo apt-get -qq install libxvidcore-dev
echo "x264"
sudo apt-get -qq install x264
echo "v41-utils"
sudo apt-get -qq install v4l-utils
echo "ffmpeg"
sudo apt-get -qq install ffmpeg
echo "cmake"
sudo apt-get -qq install cmake
echo "Done."
