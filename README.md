![](./images/FS_Logo.png)

# FS-AI_API


## NOTE:

### This *release candidate* of the **FS-AI_API** repository contains the source code for the 2021 software revision version of the IMechE FS-AI ADS-DV.

It has been provided as a reference for teams to start implementing their vehicle interface code.

Previously the 2019 interface was provided as a library 'libfs-ai_api_amd64.a' intended for static linking.

Source code for the library is now released under the MIT license so it may be compiled for any Linux architecture and modified if required.

The 2021 FS-AI ADS-DV has an updated CAN interface to support the new vehicle feature of modulated hydraulic brakes. This interface is backward compatible to the 2019 vehicle.

Further work has been done to improve the Console & Tester example programmes.




## Contents

```
\Docs
	(documentation files)
\FS-AI_API
	(library source code & makefile)
\FS-AI_API_Console
	(Console test programme)
\FS-AI_API_Tester
	(Test programme)
\images
	(images for Markdown files)
```



To build:

`cd FS-AI_API_Tester` or `cd FS-AI_API_Console`
`make` to build (library will build / rebuild as needed).

`./fs-ai_api_tester vcan0` to run on vcan0.

`./fs-ai_api_tester can0` to run on can0, etc.

NOTE: See `setup.sh` for information on configuring a CAN interface under Linux and the commands to setup can0, vcan0, etc.




## How to use
The library contains 7 functions plus associated data structures and enums:



### `int fs-ai_api_init(char *CAN_interface, int debug, int simulate);`
Accepts a character string referring to an available CAN interface (e.g. can0) and initialises the library.

Setting `debug` to a non-zero value outputs debugging info to `stdout` (otherwise the library is silent).

Setting `simulate` to a non-zero value causes simulated values to be loaded into the data structures.

NOTE: The simulated data is not representative of vehicle operation and simply changes on each call of `fs_ai_api_vcu2ai_get_data()`.




### `void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data);`
Populates an instance of the `fs_ai_api_vcu2ai` data structure with the latest values received from the vehicle. Data receives are asynchronous - each CAN frame is buffered as it is received.




### `void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data);`
Transmits the CAN frames associated with the data passed in via the `fs_ai_api_ai2vcu` data structure.

NOTE: This function must be called frequently enough to prevent the CAN timeout diagnostics of the vehicle ECU from triggering. However calling the function too frequently could overload the CAN bus so an internal timer prevents re-transmission of CAN frames at a period less than approx. 8ms. Calling this function every 10ms is optimal, data updates will be sent to the vehicle as fast as possible.




### `void fs_ai_api_imu_get_data(fs_ai_api_imu *data);`

Populates an instance of the `fs_ai_api_imu` data structure with the latest values received from the PCAN-GPS fitted to the vehicle. Data receives are asynchronous - each CAN frame is buffered as it is received.




### `void fs_ai_api_gps_get_data(fs_ai_api_gps *data);`

Populates an instance of the `fs_ai_api_gps` data structure with the latest values received from the PCAN-GPS fitted to the vehicle. Data receives are asynchronous - each CAN frame is buffered as it is received.



### `void fs_ai_api_get_can_stats(can_stats_t *data);`

Populates an instance of the `can_stats_t` data structure with the latest values received from the vehicle. Use this for debugging CAN receives.



### `void fs_ai_api_clear_can_stats();`

Clears the `can_stats_t` data structure.


## 




## Further Documentation
Please refer to the full specification document for full information on the FS-AI ADS-DV CAN interface: [ADS-DV Software Interface Specification-v4.0.pdf](./Docs/ADS-DV_Software_Interface_Specification_v4.0.pdf).

(also published on the IMechE website as: https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/ads-dv-software-interface-specification-v4-0.pdf?sfvrsn=2 ).

The referenced CAN database can be found here: [ADSDV_2021_VCU_AI_interface_v2.dbc](./Docs/ADSDV_2021_VCU_AI_interface_v2.dbc).

This software library exposes only those aspects of the full interface that are deemed essential for the 2021 Formula Student AI DDT competition using the FS-AI ADS-DV.

