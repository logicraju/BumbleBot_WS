*Protothreads* are extremely lightweight stackless threads designed for
severely memory constrained systems. 

Protothreads provides a blocking context on top of an event-driven
system, without the overhead of per-thread stacks. The purpose of
protothreads is to implement sequential flow of control without
complex state machines or full multi-threading.

Main features:

    * No machine specific code - the protothreads library is pure C
    * Does not use error-prone functions such as longjmp()
    * Very small RAM overhead - only two bytes per protothread
    * Provides blocking wait without full multi-threading or
      stack-switching
    * Freely available under a BSD-like open source license    

The protothreads library is released under an open source BSD-style
license that allows for both non-commercial and commercial usage. The
only requirement is that credit is given.

The protothreads library was written by Adam Dunkels <adam@sics.se>
with support from Oliver Schmidt <ol.sc@web.de>. Arduino port by 
Ben Artin <ben@artins.org>

More information and new versions can be found at the protothreads
homepage: http://www.sics.se/~adam/pt/

The Arduino port lives at: https://gitlab.com/airbornemint/arduino-protothreads

Documentation can be found in the doc/ subdirectory.

Two Aruino example programs are included to illustrate the use of protothreads:

 * Blink: Blinks the on-board LED. 
 * Button: Turns the on-board LED on an off with a push button.

Both examples are based on stock Arduino sample code, modified to work with 
protothreads.

Ben Artin, July 15 2020

Based on work by Adam Dunkels, 3 June 2006
