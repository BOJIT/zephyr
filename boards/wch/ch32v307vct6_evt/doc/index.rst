.. zephyr:board:: ch32v307vct6_evt

Overview
********

The `WCH`_ CH32V307VCT6-EVT hardware provides support for QingKe V4F 32-bit RISC-V
processor.

The `WCH webpage on CH32V307`_ contains
the processor's information and the datasheet.

Hardware
********

TODO

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

Applications for the ``ch32v307vct6_evt`` board target can be built and flashed
in the usual way (see :ref:`build_an_application` and :ref:`application_run`
for more details); however, an external programmer (like the `WCH LinkE`_) is required since the board
does not have any built-in debug support.

Flashing
========

You can use ``minichlink`` to flash the board. Once ``minichlink`` has been set
up, build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ch32v307vct6_evt
   :goals: build flash

Debugging
=========

This board can be debugged via OpenOCD using the WCH openOCD liberated fork, available at https://github.com/jnk0le/openocd-wch.

References
**********

.. target-notes::

.. _WCH: http://www.wch-ic.com
.. _WCH webpage on CH32V307: https://www.wch-ic.com/products/CH32V307.html
.. _WCH LinkE: https://www.wch-ic.com/products/WCH-Link.html
