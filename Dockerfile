FROM ubuntu:16.04

RUN apt-get update
RUN apt-get -y install python3.5 python3-pip python3-pyaudio python3-numpy python3-requests git
RUN pip3 install apiai
