# I/NAV Page Layout

## Synchronization Pattern

The synchronization pattern allows the receiver to achieve synchronization to the page boundary.

## Tail Bits

The tail bits field consists of 6 zero-value bits enabling completion of the FEC decoding of each page’s information content in the user receiver.

## I/NAV Page Part

The structure of the nominal I/NAV even and odd page parts on E5b-I and E1-B are defined in the tables below. A nominal page is composed by the two-page parts (even and odd) transmitted sequentially over the same frequency (“vertical page”).
## I/NAV Nominal Page with Bits Allocation

| Even/ odd=1 | Page Type | Data j (2/2) | OSNMA | SAR | Spare | CRC_j | SSP | Tail |
| ----------- | --------- | ------------ | ----- | --- | ----- | ----- | --- | ---- |
| 1           | 1         | 16           | 40    | 22  | 2     | 24    | 8   | 6    |

| Even/ odd=0 | Page Type | Data k (1/2) | Tail |
| ----------- | --------- | ------------ | ---- |
| 1           | 1         | 112          | 6    |

The parameters for the nominal page have the following meaning and related values:
- Even/Odd field (1 bit) to indicate the part of the page (0=even/1=odd) that is broadcast
- Page Type (1 bit) equal to 0 to indicate the nominal page type
- Data field composed of a nominal word (described in 4.3.5) of 128 bits (comprising 112 bits of data (1/2) and 16 bits of data (2/2))
- OSNMA protocol data (40 bits) on E1-B only: the cryptographic data for Galileo Open Service Navigation Message Authentication and the OSNMA message structure are defined in [RD3]
- SAR data (22 bits) composed of SAR RLM data on E1-B only as defined in 4.3.8
- CRC (24 bits) is computed
	- on Even/Odd fields, Page Type fields, Data fields (1/2 and 2/2) and Reserved 1 field for I/NAV nominal pages on E5b-I;
	- on Even/Odd fields, Page Type fields, Data fields (1/2 and 2/2), OSNMA field, SAR field and Spare field for I/NAV nominal pages on E1B.

In nominal mode the CRC is computed for the Even and Odd parts of a page of the same frequency (“vertical CRC”) and is always broadcast on the second part of the “vertical page”. More details on the computation of the checksum are provided below.
- SSP (8 bits) containing one of the three secondary synchronisation pattern configurations as defined
- Tail bits (2*\6 bits) as defined above. These fielda re not protected by the CRC
## Checksum

The checksum, which employs a CRC technique, is used to detect the reception of corrupted data. The checksum does not include the frame synchronization pattern or the tail bit fields since these do not form part of the required message information. For the F/NAV and I/NAV data, a CRC of 24 bits is generated from the generator polynomial G(X) described below:

\(G(X)=(1+X)P(X)\)

\(P(X)\) is a primitive and irreducible polynomial given by the following equation.

\(P(X)=X^{23}+X^{17}+X^{13}+X^{12}+X^{11}+X^{9}+X^{8}+X^{7}+X^{5}+X^{3}+1\)

The CRC is composed of a sequence of 24 parity bits \(p_{i}\); for any \(i\) from 1 to 24, \(p_{i}\) is the coefficient of \(X^{24-i}\) in \(R(X)\) where:
- \(R(X)\) is the remainder of the binary polynomial algebra division of the polynomial \(m(X) X^{24}\) by \(G(X)\), and
- \(m(X)=m_{1}X^{k-1}+\dots+m_{k-2}X^{2}+m_{k-1}X+m_{k}\) with \(m_{1}, m_{2}, \dots, m_{k}\) the sequence of \(k-\) bits information to be protected by the CRC, and \(m_{1}\) as the MSB.


## Secondary Synchronization Pattern

The Secondary Synchronisation Pattern (SSP) is a pre-defined data bit sequence of 8 bits located at the end of the I/NAV pages. The SSP can assume three different configurations, named SSP1, SSP2 and SSP3.

Before FEC encoding, the SSP is inserted between the CRC and the tail bits in the I/NAV pages on E1, cycling among the three configurations as shown in Table 38, thus maintaining synchronisation to (GST0 + 1 s) modulo 6 seconds.

After FEC encoding, the last 16 symbols of E1 I/NAV pages provide three pre-defined sequences with well-defined cross-correlation properties. These are shown in the table below.


|                                                                                                                              | SSP1             | SSP2             | SSP3             |
| ---------------------------------------------------------------------------------------------------------------------------- | ---------------- | ---------------- | ---------------- |
| Plain SSP configurations                                                                                                     | 00000100         | 00101011         | 00101111         |
| Encoded SSP configurations (last 16 symbols of the I/NAV E1 pages, after FEC encoding of the 8 plain SSP bits + 6 tail bits) | 1110100100100101 | 0110110001001110 | 1101000000111110 |

The Secondary Synchronisation Pattern allows a receiver to correlate a local replica of the encoded pattern configurations with the received data symbols, after de-interleaving. As soon as any occurrence of the three SSP configurations is detected in the incoming symbol stream, the receiver has the knowledge of the GST (modulo 6 seconds) without the need for successful data decoding.

Note: SSP configurations are an evolution of the Galileo navigation message and as such will be gradually deployed to all Galileo satellites, including those already in orbit. During this deployment phase the not-yet upgraded satellites will be downlinking a legacy reserved sequence.

SSP configurations will not be downlinked while I/NAV dummy messages are transmitted on E1-B.

In both cases above, the contents transmitted instead of the SSP configurations are not intended for time synchronisation and have low cross-correlation with the SSP configurations provided in the table above.

## I/NAV Alert Page with Bits Allocation

The structure of the alert I/NAV even and odd page parts on E5b-I and E1-B are defined in Table 37. An alert page is composed by the two-page parts (even and odd) transmitted at the same epoch over E5b-I and E1-B (“horizontal page”).

| Even/ odd=1 | Page Type | Reserved 1 (2/2) | CRC | SSP | Tail |
| ----------- | --------- | ---------------- | --- | --- | ---- |
| 1           | 1         | 80               | 24  | 8   | 6    |

| Even/ odd=0 | Page Type | Reserved 1 (1/2) | Tail |
| ----------- | --------- | ---------------- | ---- |
| 1           | 1         | 112              | 6    |

The parameters for the alert page have the following meaning and related values:
- Even/Odd field to indicate the part of the page (0=even/1=odd) that is broadcast
- Page Type (1 bit) equal to 1 to indicate the alert page type
- CRC (24 bits) computed on the Even/Odd fields, Page Type fields and on “Reserved 1” (1/2 and 2/2). In alert mode the CRC is computed for the Even/ Odd pages of both frequencies E5b and E1-B (“horizontal CRC”).

More details on the computation of the checksum are provided in Section 
5.1.9.4

- The “Reserved 1” and “Reserved 2” fields will be published in a future update of this ICD. Note that the Reserved 2 field on E5b-I and the SSP field on E1-B are not protected by CRC.
- SSP (8 bits) containing one of the three secondary synchronisation pattern configurations as defined above.
- Tail bits (2\*6 bits) as defined above. These fields are not protected by the CRC.

# I/NAV Nominal Sub-Frame Layout

In the nominal mode, the page sequence for I/NAV E5b-I and I/NAV E1-B components in every sub-frame is according to the table below, where T0 is synchronised with GST origin modulo 30 seconds.


| T0 (GST0 sync.) (s) | E5b-I Sub frame ID | E5b-I Page | E5b-I Content                      | E1-B Content                                 | E1-B Page | E1-B Sub frame ID |
| ------------------- | ------------------ | ---------- | ---------------------------------- | -------------------------------------------- | --------- | ----------------- |
| 0                   | N                  | Even       | Word 1 (1/2)                       | Word 16 (2/2), OSNMA, SAR, CRC, SSP3         | Odd       | N-1               |
| 1                   | N                  | Odd        | Word 1 (2/2), Res, CRC, Res        | Word 2 (1/2)                                 | Even      | N                 |
| 2                   | N                  | Even       | Word 3 (1/2)                       | Word 2 (2/2), OSNMA, SAR, CRC, SSP1          | Odd       | N                 |
| 3                   | N                  | Odd        | Word 3 (2/2), Res, CRC, Res        | Word 4 (1/2)                                 | Even      | N                 |
| 4                   | N                  | Even       | Word 5 (1/2)                       | Word 4 (2/2), OSNMA, SAR, CRC, SSP2          | Odd       | N                 |
| 5                   | N                  | Odd        | Word 5 (2/2), Res, CRC, Res        | Word 6 (1/2)                                 | Even      | N                 |
| 6                   | N                  | Even       | Word 7 or 9 (1/2)*                 | Word 6 (2/2), OSNMA, SAR, CRC, SSP3          | Odd       | N                 |
| 7                   | N                  | Odd        | Word 7 or 9 (2/2)*, Res, CRC, Res  | Word 7 or 9 (1/2)*                           | Even      | N                 |
| 8                   | N                  | Even       | Word 8 or 10 (1/2)*                | Word 7 or 9 (2/2)*, OSNMA, SAR, CRC, SSP1    | Odd       | N                 |
| 9                   | N                  | Odd        | Word 8 or 10 (2/2)*, Res, CRC, Res | Word 8 or 10 (1/2)*                          | Even      | N                 |
| 10                  | N                  | Even       | Word 0 (1/2)                       | Word 8 or 10 (2/2)*, OSNMA, SAR, CRC, SSP2   | Odd       | N                 |
| 11                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 17 or 18 (1/2)**                        | Even      | N                 |
| 12                  | N                  | Even       | Word 0 (1/2)                       | Word 17 or 18 (2/2)**, OSNMA, SAR, CRC, SSP3 | Odd       | N                 |
| 13                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 19 or 20 (1/2)**                        | Even      | N                 |
| 14                  | N                  | Even       | Word 0 (1/2)                       | Word 19 or 20 (2/2)**, OSNMA, SAR, CRC, SSP1 | Odd       | N                 |
| 15                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 16 (1/2)                                | Even      | N                 |
| 16                  | N                  | Even       | Word 0 (1/2)                       | Word 16 (2/2), OSNMA, SAR, CRC, SSP2         | Odd       | N                 |
| 17                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 0 (1/2)                                 | Even      | N                 |
| 18                  | N                  | Even       | Word 0 (1/2)                       | Word 0 (2/2), OSNMA, SAR, CRC, SSP3          | Odd       | N                 |
| 19                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 22 (1/2)                                | Even      | N                 |
| 20                  | N                  | Even       | Word 2 (1/2)                       | Word 22 (2/2), OSNMA, SAR, CRC, SSP1         | Odd       | N                 |
| 21                  | N                  | Odd        | Word 2 (2/2), Res, CRC, Res        | Word 1 (1/2)                                 | Even      | N                 |
| 22                  | N                  | Even       | Word 4 (1/2)                       | Word 1 (2/2), OSNMA, SAR, CRC, SSP2          | Odd       | N                 |
| 23                  | N                  | Odd        | Word 4 (2/2), Res, CRC, Res        | Word 3 (1/2)                                 | Even      | N                 |
| 24                  | N                  | Even       | Word 6 (1/2)                       | Word 3 (2/2), OSNMA, SAR, CRC, SSP3          | Odd       | N                 |
| 25                  | N                  | Odd        | Word 6 (2/2), Res, CRC, Res        | Word 5 (1/2)                                 | Even      | N                 |
| 26                  | N                  | Even       | Word 0 (1/2)                       | Word 5 (2/2), OSNMA, SAR, CRC, SSP1          | Odd       | N                 |
| 27                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 0 (1/2)                                 | Even      | N                 |
| 28                  | N                  | Even       | Word 0 (1/2)                       | Word 0 (2/2), OSNMA, SAR, CRC, SSP2          | Odd       | N                 |
| 29                  | N                  | Odd        | Word 0 (2/2), Res, CRC, Res        | Word 16 (1/2)                                | Even      | N                 |
| 30                  | N+1                | Even       | Word 1 (1/2)                       | Word 16 (2/2), OSNMA, SAR, CRC, SSP3         | Odd       | N                 |
\* The dissemination sequence of Word Types 7, 8, 9 and 10 within a frame is detailed in the table below.
\** FEC2 RS CED broadcast cycles through all four FEC2 RS CED Words.
\*** The scope of the figure is solely to show the sequence of the information within the full sub-frame. At this scope, not all the fields are represented

The sub-frame structure shown in the table above is indicative: Deviations from the I/NAV word dissemination sequence as shown may appear, individually per satellite and precautions as per Section 4.1.2 are expected to be foreseen within the user receiver.

The indication of Word Type 0 in the table above reflects spare capacities in the I/NAV word dissemination sequence reserved for future use.

## I/NAV Nominal Frame Layout


## I/NAV Word Types

The content of the I/NAV word types is stated in the following tables (see Chapter 5 for a description of the I/NAV word types contents).

### Word Type 1: Ephemeris (1/4)

| Parameter      | Type=1 | IOD_nav | toe | M0  | e   | sqrtA | Reserved |
| -------------- | ------ | ------- | --- | --- | --- | ----- | -------- |
| Number of bits | 6      | 10      | 14  | 32  | 32  | 32    | 2        |

### Word Type 2: Ephemeris (2/4)

| Parameter      | Type=2 | IOD_nav | Omega0 | i0  | omega | i_dot | Reserved |
| -------------- | ------ | ------- | ------ | --- | ----- | ----- | -------- |
| Number of bits | 6      | 10      | 32     | 32  | 32    | 14    | 2        |

### Word Type 3: Ephemeris (3/4) and SISA

| Parameter      | Type=3 | IOD_nav | Omega_dot | Delta_n | Cuc | Cus | Crc | Crs | SISA(E1,E5b) |
| -------------- | ------ | ------- | --------- | ------- | --- | --- | --- | --- | ------------ |
| Number of bits | 6      | 10      | 24        | 16      | 16  | 16  | 16  | 16  | 8            |

### Word Type 4: SVID, Ephemeris (4/4), and Clock Correction Parameters

| Parameter      | Type=4 | IOD_nav | SVID | Cic | Cis | toc | alpha0 | alpha1 | alpha2 | Spare |
| -------------- | ------ | ------- | ---- | --- | --- | --- | ------ | ------ | ------ | ----- |
| Number of bits | 6      | 10      | 6    | 16  | 16  | 14  | 31     | 21     | 6      | 2     |

### Word Type 5: Ionospheric Correction, BGD, Signal Health, and Data Validity Status and GST

| Parameter      | Type=5 | ai0 | ai1 | ai2 | Region 1 | Region 2 | Region 3 | Region 4 | Region 5 | BGD(E1,E5a) | BGD(E1,E5b) | E5b_HS | E1B_HS | E5b_DVS | E5B_DVS | WN  | TOW | Spare |
| -------------- | ------ | --- | --- | --- | -------- | -------- | -------- | -------- | -------- | ----------- | ----------- | ------ | ------ | ------- | ------- | --- | --- | ----- |
| Number of bits | 6      | 11  | 11  | 14  | 1        | 1        | 1        | 1        | 1        | 10          | 10          | 2      | 2      | 1       | 1       | 12  | 20  | 23    |

### Word Type 6: GST-UTC Conversion Parameters

| Parameter      | Type=6 | A0  | A1  | Delta_t_LS | t_ot | WN_0t | W_N_LSF | DN  | Delta_t_LSF | TOW | Spare |
| -------------- | ------ | --- | --- | ---------- | ---- | ----- | ------- | --- | ----------- | --- | ----- |
| Number of bits | 6      | 32  | 24  | 8          | 8    | 8     | 8       | 3   | 8           | 20  | 3     |

### Word Type 7: Almanac for SVID1 (1/2), Almanac Reference Time and Almanac Reference Week Number

| Parameter      | Type=7 | IOD_a | WN_a | t0_a | SVID1 | Delta_sqrtA | e   | omega | delta_i | Omega0 | Omega_dot | M0  | Reserved |
| -------------- | ------ | ----- | ---- | ---- | ----- | ----------- | --- | ----- | ------- | ------ | --------- | --- | -------- |
| Number of bits | 6      | 4     | 2    | 10   | 6     | 13          | 11  | 16    | 11      | 16     | 11        | 16  | 6        |

### Word Type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)

| Parameter      | Type=8 | IOD_a | alpha0 | alpha1 | E5b_HS | E1B_HS | SVID2 | Delta_sqrtA | e   | omega | delta_i | Omega0 | Omega_dot | Spare |
| -------------- | ------ | ----- | ------ | ------ | ------ | ------ | ----- | ----------- | --- | ----- | ------- | ------ | --------- | ----- |
| Number of bits | 6      | 4     | 16     | 13     | 2      | 2      | 6     | 13          | 11  | 16    | 11      | 16     | 11        | 1     |

### Word Type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)

| Parameter      | Type=9 | IOD_a | WN_a | t0_a | M0  | alpha0 | alpha1 | E5b_HS | E1B_HS | SVID3 | Delta_sqrtA | e   | omega | delta_i |
| -------------- | ------ | ----- | ---- | ---- | --- | ------ | ------ | ------ | ------ | ----- | ----------- | --- | ----- | ------- |
| Number of bits | 6      | 4     | 2    | 10   | 16  | 16     | 13     | 2      | 2      | 6     | 13          | 11  | 16    | 11      |

### Word Type 10: Almanac for SVID3 (2/2) and GST-GPS Conversion Parameters

| Parameter      | Type=10 | IOD_a | Omega0 | Omega_dot | M0  | alpha0 | alpha1 | E5b_HS | E1B_HS | A0_G | A1_G | t0_G | WN0_G |
| -------------- | ------- | ----- | ------ | --------- | --- | ------ | ------ | ------ | ------ | ---- | ---- | ---- | ----- |
| Number of bits | 6       | 4     | 16     | 11        | 16  | 16     | 13     | 2      | 2      | 16   | 12   | 8    | 6     |

### Word Type 16: Reduced Clock and Ephemeris Data (CED) Parameters

| Parameter      | Type=16 | Delta_A_red | e_x_red | e_y_red | Delta_i0_red | Omega0_red | lambda0_red | alpha0_red | alpha1_red |
| -------------- | ------- | ----------- | ------- | ------- | ------------ | ---------- | ----------- | ---------- | ---------- |
| Number of bits | 6       | 5           | 13      | 13      | 17           | 23         | 23          | 22         | 6          |

# Ephemeris Parameters


| Parameter | Definition                                                                   | Bits | Scale factor | Unit             |
| --------- | ---------------------------------------------------------------------------- | ---- | ------------ | ---------------- |
| M0        | Mean anomaly at reference time                                               | 32*  | 2e-31        | semi-circles**   |
| Delta_n   | Mean motion difference from computed value                                   | 16*  | 2e-43        | semi-circles/s** |
| e         | Eccentricity                                                                 | 32   | 2e-33        | N/A              |
| sqrtA     | Square root of the semi-major axis                                           | 32   | 2e-19        | meter^{1/2}      |
| Omega0    | Longitude of ascending node of orbital plane at weekly epoch**               | 32*  | 2e-31        | semi-circles**   |
| i0        | Inclination angle at reference time                                          | 32*  | 2e-31        | semi-circles**   |
| omega     | Argument of perigee                                                          | 32*  | 2e-31        | semi-circles**   |
| Omega_dot | Rate of change of right ascension                                            | 24*  | 2e-43        | semi-circles/s** |
| i_dot     | Rate of change of inclination angle                                          | 14*  | 2e-43        | semi-circles/s** |
| Cuc       | Amplitude of the cosine harmonic correction term to the argument of latitude | 16*  | 2e-29        | radians          |
| Cus       | Amplitude of the sine harmonic correction term to the argument of latitude   | 16*  | 2e-29        | radians          |
| Crc       | Amplitude of the cosine harmonic correction term to the orbit radius         | 16*  | 2e-5         | radians          |
| Crs       | Amplitude of the sine harmonic correction term to the orbit radius           | 16*  | 2e-5         | radians          |
| Cic       | Amplitude of the cosine harmonic correction term to the angle of inclination | 16*  | 2e-29        | radians          |
| Cis       | Amplitude of the sine harmonic correction term to the angle of inclination   | 16*  | 2e-29        | radians          |
| toe       | Ephemeris reference time                                                     | 14   | 60           | seconds          |
| WN        | Week Number                                                                  | 12   | 1            | week             |
| TOW       | Time of Week                                                                 | 20   | 1            | seconds          |
| toc       | Clock correction data reference Time of Week                                 | 14   | 60           | seconds          |
| alpha0    | SV clock bias correction coefficient                                         | 31*  | 2e-34        | seconds          |
| alpha1    | SV clock drift correction coefficient                                        | 21*  | 2e-46        | seconds/s        |
| alpha2    | SV clock drift rate correction coefficient                                   | 6*   | 2e-59        | seconds/s^2      |

\* Parameters so indicated are two’s complement, with the sign bit (+ or −) occupying the MSB.
\** Note that the ‘semi-circle’ is not a SI unit but can be converted as: 1 semi-circle = π rad.
\*** More precisely, Omega0 is the longitude of ascending node of orbital plane at the weekly epoch propagated to the reference time 𝑡o𝑒 at the rate of change of right ascension.

