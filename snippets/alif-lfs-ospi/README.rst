.. _snippet-alif-lfs-ospi:

Alif LittleFS support over OSPI Flash
#####################################

Overview
********

This snippet enables the Zephyr LittleFS sample to run with Alif MRAM and
OSPI Flash storage nodes.

The LittleFS sample supports multiple storage nodes through
``zephyr,fstab``. For Alif development kits, this snippet creates one
LittleFS storage node on MRAM and one LittleFS storage node on OSPI Flash.

The snippet:

* enables the Alif OSPI flash driver configuration from
  ``alif-lfs-ospi.conf``;
* selects the required OSPI pinctrl and flash overlay for the target board;
* enables the OSPI flash device node;
* provides MRAM and OSPI Flash LittleFS mount information through
  ``zephyr,fstab``.

The Balletto board family uses ``alif_balletto_dk_rtss_ospi.overlay`` and
the Ensemble board family uses ``alif_ensemble_dk_rtss_ospi.overlay``.

Storage Layout
**************

The overlay updates the selected flash device to OSPI flash and creates
two LittleFS storage nodes:

* ``/lfs1`` on MRAM storage;
* ``/lfs2`` on OSPI Flash storage.

.. code-block:: devicetree

   chosen {
           zephyr,flash-controller = &ospi_flash;
           zephyr,flash = &flash_storage;
   };

The existing MRAM code partition is left unchanged. Alif development kit
MRAM is already split into multiple partitions, and this snippet only adds
the LittleFS storage nodes used by the sample.

The sample discovers these nodes from ``zephyr,fstab`` and runs the
LittleFS test flow on each configured mount point.

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

