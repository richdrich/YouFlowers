YouFlowers
==========

This app supports an art project I've created that will be shown for the first time, I hope, at Burning Seed (Australia's Burning Man regional).

The concept is that we have a collection of flowers on stalks which turn to follow passers by, much as sunflowers follow the sun.
 
It uses OpenCV to track the largest moving object through two cameras which (by line-of-position solving) yield an XY coordinate of the person. That then tells the flowers (which are on servos driven by an Arduino USB) where to move to.

The code is written in C++ and is a bit grungy. It kind of grew through experiments over a couple of years. It needs a fairly grunty machine to run - I spent much time trying to optimize it to run on a Beagleboard, but there just aren't enough clocks.

You are very welcome to use this for anything you want on a non-commercial basis, but cloning my art exactly would be a boring thing to do, really.
 