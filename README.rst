******************
mikroBUS Click ID Flasher
******************


Overview
########
This repository contains the source for a mikroBUS Click ID EEPROM flasher utility which helps to write a particular mikroBUS add-on board  manifest
binary to the 1-wire EEPROM on-board the Click ID adapter or ID EEPROM enabled Clicks

Building and Running
####################

First, ensure that all required tools are installed by following Zephyr's
`Getting Started Guide <https://docs.zephyrproject.org/latest/getting_started/index.html>`_.

Next, clone and synchronize repositories.

.. code-block:: bash

    west init -m https://github.com/vaishnav98/mikrobus-clickid-flasher zephyrproject
    cd zephyrproject
    west update


Next, update git submodules. Currently, there is a git submodule for the
``manifesto`` utility.

.. code-block:: bash

    cd greybus
    git submodule init
    git submodule update
    cd ..

Currently the only the CC1352 based boards are supported :

.. code-block:: bash

    west build -b beagleconnect_freedom -t flash ../mikrobus/samples/subsys/mikrobus/flasher -DCONFIG_MIKROBUS_FLASHER_CLICK_NAME=\"RTC-6-CLICK\"
