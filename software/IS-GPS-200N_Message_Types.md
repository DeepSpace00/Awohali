# Message Type 10 - Ephemeris 1

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 52        | 53        | 54        | 55  | 66           | 71  | 82      | 108   | 133      | 150          | 173 | 206 | 239   | 272                   | 273         | 274      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | --------- | --------- | --------- | --- | ------------ | --- | ------- | ----- | -------- | ------------ | --- | --- | ----- | --------------------- | ----------- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | WN  | L1 Health | L2 Health | L5 Health | top | URA_ED index | toe | Delta_A | A_dot | Delta_n0 | Delta_n0_dot | M0  | e   | omega | Integrity Status Flag | L2C Phasing | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 13  | 1         | 1         | 1         | 11  | 5            | 11  | 26      | 25    | 17       | 23           | 33  | 33  | 33    | 1                     | 1           | 3        | 24  |

# Message Type 11 - Ephemeris 2

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50     | 83  | 116             | 133  | 148 | 164 | 180 | 204 | 228 | 249 | 270      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | ------ | --- | --------------- | ---- | --- | --- | --- | --- | --- | --- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | toe | Omega0 | i0  | Delta_Omega_dot | IDOT | Cis | Cic | Crs | Crc | Cus | Cuc | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 33     | 33  | 17              | 15   | 16  | 16  | 24  | 24  | 21  | 21  | 7        | 24  |

# Message Type 13 - Clock Differential Correction

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50   | 61           | 62  | 96           | 97  | 131          | 132 | 166          | 167 | 201          | 202 | 236          | 237 | 271      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | ---- | ------------ | --- | ------------ | --- | ------------ | --- | ------------ | --- | ------------ | --- | ------------ | --- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | top | t_OD | DC Data Type | CDC | DC Data Type | CDC | DC Data Type | CDC | DC Data Type | CDC | DC Data Type | CDC | DC Data Type | CDC | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 11   | 1            | 34  | 1            | 34  | 1            | 34  | 1            | 34  | 1            | 34  | 1            | 34  | 6        | 24  |

# Message Type 14 - Ephemeris Differential Correction

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50   | 61           | 62  | 154          | 155 | 247      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | ---- | ------------ | --- | ------------ | --- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | top | t_OD | DC Data Type | EDC | DC Data Type | EDC | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 11   | 1            | 92  | 1            | 92  | 30       | 24  |

# Message Type 30 - Clock, IONO & Group Delay
| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50             | 55             | 58             | 61  | 72  | 98  | 118 | 128 | 141       | 154     | 167      | 180      | 193    | 201    | 209    | 219    | 217    | 225   | 233   | 241   | 249   | 257   | 265      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | -------------- | -------------- | -------------- | --- | --- | --- | --- | --- | --------- | ------- | -------- | -------- | ------ | ------ | ------ | ------ | ------ | ----- | ----- | ----- | ----- | ----- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | top | URA_NED0 Index | URA_NED1 Index | URA_NED2 Index | toc | af0 | af1 | af2 | TGD | ISC_L1C/A | ISC_L2C | ISC_L5I5 | ISC_L5Q5 | alpha0 | alpha1 | alpha2 | alpha3 | alpha3 | beta0 | beta1 | beta2 | beta3 | WN_OP | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 5              | 3              | 3              | 11  | 26  | 20  | 10  | 13  | 13        | 13      | 13       | 13       | 8      | 8      | 8      | 8      | 8      | 8     | 8     | 8     | 8     | 8     | 12       | 24  |

# Message Type 32 - Clock & EOP

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50             | 55             | 58             | 61  | 72  | 98  | 118 | 128   | 144  | 165      | 180  | 201      | 216         | 247             | 266      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | -------------- | -------------- | -------------- | --- | --- | --- | --- | ----- | ---- | -------- | ---- | -------- | ----------- | --------------- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | top | URA_NED0 Index | URA_NED1 Index | URA_NED2 Index | toc | af0 | af1 | af2 | t_EOP | PM_X | PM_dot_X | PM_Y | PM_dot_Y | Delta_UTGPS | Delta_U_dot_GPS | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 5              | 3              | 3              | 11  | 26  | 20  | 10  | 16    | 21   | 15       | 21   | 15       | 31          | 19              | 11       | 24  |

# Message Type 33 - Clock & UTC

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50       | 55       | 58       | 61  | 72  | 98  | 118 | 128 | 144 | 157 | 164        | 172  | 188   | 201    | 214 | 218         | 226      | 277 |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | -------- | -------- | -------- | --- | --- | --- | --- | --- | --- | --- | ---------- | ---- | ----- | ------ | --- | ----------- | -------- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | top | URA_NED0 | URA_NED1 | URA_NED2 | toc | af0 | af1 | af2 | A0  | A1  | A2  | Delta_t_LS | t_ot | WN_ot | WN_LSF | DN  | Delta_t_LSF | RESERVED | CRC |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 5        | 3        | 3        | 11  | 26  | 20  | 10  | 16  | 13  | 7   | 8          | 16   | 13    | 13     | 4   | 8           | 51       | 24  |

# Message Type 34 - Clock & Differential Correction

| Bit Index      | 1        | 9       | 15              | 21                | 38         | 39  | 50       | 55       | 58       | 61  | 72  | 98  | 118 | 128  | 139  | 150          | 151 | 185 | 277 |     |
| -------------- | -------- | ------- | --------------- | ----------------- | ---------- | --- | -------- | -------- | -------- | --- | --- | --- | --- | ---- | ---- | ------------ | --- | --- | --- | --- |
| Parameter      | Preamble | PRN/SVN | Message Type ID | Message TOW Count | Alert Flag | top | URA_NED0 | URA_NED1 | URA_NED2 | toc | af0 | af1 | af2 | t_op | t_OD | DC Data Type | CDC | EDC | CRC |     |
| Number of Bits | 8        | 6       | 6               | 17                | 1          | 11  | 5        | 3        | 3        | 11  | 26  | 20  | 10  | 11   | 11   | 1            | 34  | 92  | 24  | 7   |
