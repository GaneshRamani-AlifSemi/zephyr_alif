.. _snippet-alif-lfs-ospi:

Alif LittleFS support over OSPI Flash
#####################################

Overview
********

This snippet enables the Zephyr LittleFS sample to use OSPI Flash as the
selected flash device while linking the application image into Alif MRAM.

The LittleFS sample discovers its storage through ``zephyr,fstab``. For Alif
development kits, this snippet creates a LittleFS storage node on OSPI Flash
and forces the Zephyr application image address to the MRAM code partition.

The snippet:

* enables the Alif OSPI flash driver configuration from
  ``alif-lfs-ospi.conf``;
* selects the required OSPI pinctrl and flash overlay for the target board;
* enables the OSPI flash device node;
* selects OSPI Flash as ``zephyr,flash``;
* keeps ``zephyr,code-partition`` pointing at the MRAM image slot and overrides
  ``CONFIG_FLASH_BASE_ADDRESS`` so the Zephyr image still links into MRAM;
* provides OSPI Flash LittleFS mount information through ``zephyr,fstab``.

The Balletto board family uses ``alif_balletto_dk_rtss_ospi.overlay`` and
the Ensemble board family uses ``alif_ensemble_dk_rtss_ospi.overlay``.

Storage Layout
**************

The overlay selects OSPI Flash as the default flash device and creates one
LittleFS storage node:

* ``/lfs`` on OSPI Flash storage.

.. code-block:: devicetree

   chosen {
           zephyr,flash-controller = &ospi_flash;
           zephyr,flash = &flash_storage;
           zephyr,code-partition = &slot0_partition;
   };

Alif development kit MRAM is already split into boot and image partitions.
The snippet removes the default MRAM storage partition from the sample layout,
uses OSPI Flash for LittleFS, and sets ``CONFIG_FLASH_BASE_ADDRESS`` to the
MRAM base address so the linker still places code in MRAM.

The sample discovers this node from ``zephyr,fstab`` and runs the LittleFS
test flow on the configured mount point.

Building and Running
********************

Example command to build using ``-S``:

.. code-block:: console

   west build -b alif_b1_dk/ab1c1f4m51820hh/rtss_he -S alif-lfs-ospi samples/subsys/fs/littlefs -p

The same build can also be requested through ``SNIPPET``:

.. code-block:: console

   west build -b alif_b1_dk/ab1c1f4m51820hh/rtss_he samples/subsys/fs/littlefs -p -- -DSNIPPET=alif-lfs-ospi

For Ensemble:

.. code-block:: console

   west build -b alif_e7_dk/ae722f80f55d5xx/rtss_he -S alif-lfs-ospi samples/subsys/fs/littlefs -p

The application can be found under :zephyr_file:`samples/subsys/fs/littlefs` in the Zephyr tree.

See :zephyr:code-sample:`fs/littlefs` application details.

