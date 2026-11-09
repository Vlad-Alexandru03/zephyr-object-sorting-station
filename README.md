# Zephyr Object Sorting Station

A one-slot station that identifies objects from a camera snapshot.

The Freenove ESP32-S3 captures a fresh JPEG on request. The Arduino UNO Q
runs object recognition on Debian and the station controller on Zephyr.

## First milestone

Press Start and display the object label, `empty`, or `unknown`.

## Later milestone

Use the SO101 arm to sort recognized objects into taught drop-off locations,
then take a second snapshot to verify that the slot is empty.
