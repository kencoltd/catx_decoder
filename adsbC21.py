import datetime
import struct
LongFrameLengthBytes = 14
ShortFrameLengthBytes = 7


def CountFx( frame ):
    #Bytes = struct.unpack('2s'*(len( frame ) / 2),frame)
    FX = 0
    #print frame
    while ( int( frame[FX], 16 ) & 0x01) != 0:
        FX = FX + 1
    FX += 1
    return FX

def Trans21GPSPos( frame ):
    #print frame
    iData = int( frame[0], 16 )
    iData <<=  8
    iData |= int( frame[1], 16 )
    iData <<=  8
    iData |= int( frame[2], 16 )
    iData <<=  8
    iData |= int( frame[3], 16 )

    #//LSB = 180/power(2,25)
    fLatitude = iData * 180.0/33554432
    
    iData = int( frame[4], 16 )
    iData <<=  8
    iData |= int( frame[5], 16 )
    iData <<=  8
    iData |= int( frame[6], 16 )
    iData <<=  8
    iData |= int( frame[7], 16 )
    #//LSB = 180/power(2,25)
    fLongitude = iData*180.0/33554432
    
    return fLatitude, fLongitude

def ReadTargetAddress( frame ):
    uiData = 0
    uiData = int( frame[0], 16 )
    uiData<<= 8
    uiData |= int( frame[1], 16 )
    uiData<<= 8
    uiData |= int( frame[2], 16 )

    return uiData

def ReadModeCHei( frame ):
    return int( frame[0] + frame[1], 16 ) * 7.62

def ReadGpsHei( frame ):
    return int( frame[0] + frame[1], 16 ) * 1.905

def ReadCallCode( frame ):
    return GetStringFrom6Char( frame )

def GetStringFrom6Char( frame ):
    #print frame
    chCallCode = ''
    c = int( frame[0], 16 )
    c >>= 2 #//bit 48-43 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = int( frame[0], 16 )<<4
    c |= int( frame[1], 16 )>>4#//bit 42-37 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = int( frame[1], 16 )<<2;
    c |= int( frame[2], 16 )>>6#//bit 36-31 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = int( frame[2], 16 )#//bit 30-25 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = int( frame[3], 16 )
    c>>= 2#//bit 24-19 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = int( frame[3], 16 )<<4
    c |= int( frame[4], 16 )>>4#//bit 18-13 of 6 bytes
    chCallCode += chr( SixBits2OneChar(c&0x3f) )

    c = int( frame[4], 16 )<<2
    c |= int( frame[5], 16 )>>6#//bit 12-7 of 6 bytes
    chCallCode += chr( SixBits2OneChar(c&0x3f) )

    c = int( frame[5], 16 )#//bit 6-1 of 6 bytes
    chCallCode += chr( SixBits2OneChar(c&0x3f) )

    return chCallCode.replace( ' ','' )

def SixBits2OneChar( value ):
    if ( value > 0 ) and ( value < 27 ):
            return (value+ord('A')-1)
    else:
            return value

def ReadAirSpeed( frame ):
    uData = int( frame[0] + frame[1], 16 )
    if ( uData & 0x8000 ) != 0:
        return (uData&0x7fff) * 0.32
    else:
        return uData * 0.11304


def ReadTrueAirSpeed( frame ):
    unData = int( frame[0] + frame[1], 16 )
    #///1knot NM/H
    return unData * 0.514444

def ReadGroundSpeed( frame ):
    unData = int( frame[0] + frame[1], 16 )
    fSpeed = unData * 0.11304
    unData = int( frame[2] + frame[3], 16 )
    fHeading = unData * 0.005493
    
    return fSpeed, fHeading


if __name__ == '__main__':
	fData = open( 'ADSB.dat', 'r' )
	aLine = fData.readline()
    
	while aLine != '' :

            bData = aLine.replace('\n', '').split(' ')
            if len( bData ) != 2:
                    print 'bad data, %s' % aLine
                    continue
                
            dTime = bData[0]
            #dFrame = bData[1]
            dFrame = struct.unpack('2s'*(len( bData[1] ) / 2),bData[1])
            
            time = datetime.datetime(
            int(dTime[0:4]),
            int(dTime[4:6]),
            int(dTime[6:8]),
            int(dTime[9:11]),
            int(dTime[11:13]),
            int(dTime[13:15]),
            int(dTime[16:19]) * 1000 )
            #print time
            ga = 0.0
            ia = 0.0
            ta = 0.0
            spd = 0.0 
            heading = 0.0
            if int( dFrame[0], 16 ) != 21:
                print 'not cat21 data, %s' % aLine
                aLine = fData.readline()
                continue

            if int( dFrame[1]+dFrame[2], 16 ) != len( dFrame ):
                print 'bad data2, %s' % aLine
                aLine = fData.readline()
                continue
            
            cFX = CountFx( dFrame[3:] )

            if cFX == 0:
                print 'no FX, %s' % aLine
                aLine = fData.readline()
                continue
            else:
                #print 'FX, %d' % cFX
                pass
                
            Offset = 3 + cFX
            bFlag1 = int( dFrame[3], 16 )
            #print 'Flag 1, 0x%X' % bFlag1

            #//I021/010 //Data Source Identifier: 2 bytes
            if ( bFlag1 & 0x80 ) != 0:
                #print 'Data Source Identifier'
                Offset += 2
                pass
            
            #//I021/040 //Target Report Descriptor: 2 bytes
            if ( bFlag1 & 0x40 ) != 0:
                #print 'Target Report Descriptor'
                Offset += 2
                pass

            #//I021/030 //Time of Day: 3 bytes //ld no definition in ADS
            if ( bFlag1 & 0x20 ) != 0:
                #print 'Time of Day'
                Offset += 3
                pass

            #//I021/130 //Position in WGS-84 Co-ordinates: 8 bytes
            if ( bFlag1 & 0x10 ) != 0:
                #print 'Position in WGS-84 Co-ordinates'
                lat, lon = Trans21GPSPos( dFrame[Offset:] )
                #print lat, lon
                Offset += 8
                pass

            #//I021/080 //Target Address: 3 bytes
            if ( bFlag1 & 0x08 ) != 0:
                tAddr = ReadTargetAddress( dFrame[Offset:] )
                #print 'Target Address 0x%X' % tAddr
                Offset += 3
                pass

            #//I021/140 //Geometric iAltitude: 2 bytes
            if ( bFlag1 & 0x04 ) != 0:
                ga = ReadGpsHei( dFrame[Offset:] )
                #print 'Geometric iAltitude %d' % ga
                Offset += 2
                pass
            
            #//I021/090 //Figure of Merit: 2 bytes
            if ( bFlag1 & 0x02 ) != 0:
                #print 'Figure of Merit'
                Offset += 2
                pass

            if cFX > 1:
                bFlag2 = int( dFrame[4], 16 )
                #print 'Flag 2, 0x%X' % bFlag2
                
                #//I021/210 //Link Technology: 1 uint8
                if ( bFlag2 & 0x80 ) != 0:
                    #print 'Link Technology'
                    Offset += 1
                    pass
                
                #//I021/145 //Flight Level: 2 bytes
                if ( bFlag2 & 0x20 ) != 0:
                    fl = ReadModeCHei( dFrame[Offset:] )
                    #print 'Flight Level %d' % fl
                    Offset += 2
                    pass

                #//I021/150 //indicated Air Speed: 2 bytes
                if ( bFlag2 & 0x10 ) != 0:
                    #print 'indicated Air Speed'
                    ia = ReadAirSpeed( dFrame[Offset:] )
                    Offset += 2
                    pass
                
                #//I021/151 //true Air Speed: 2 bytes
                if ( bFlag2 & 0x08 ) != 0:
                    #print 'true Air Speed'
                    ta = ReadTrueAirSpeed( dFrame[Offset:] )
                    Offset += 2
                    pass

                #//I021/152 //Magnetic Heading: 2 bytes
                if ( bFlag2 & 0x04 ) != 0:
                    #print 'Magnetic Heading'
                    Offset += 2
                    pass

                #//I021/155 //Barometric Vertical Rate: 2 bytes
                if ( bFlag2 & 0x02 ) != 0:
                    #print 'Barometric Vertical Rate'
                    Offset += 2
                    pass

            if cFX > 2:
                bFlag3 = int( dFrame[5], 16 )
                #print 'Flag 3, 0x%X' % bFlag3
                
                #//I021/157 //Geometric Vertical Rate: 2 bytes
                if ( bFlag3 & 0x80 ) != 0:
                    #print 'Geometric Vertical Rate'
                    Offset += 2
                    pass
                
                #//I021/160 //Ground Vector: 4 bytes
                if ( bFlag3 & 0x40 ) != 0:
                    #print 'Ground Vector'
                    spd, heading = ReadGroundSpeed( dFrame[Offset:] )
                    Offset += 4
                    pass

                #//I021/SPARE //Spare bits set to zero
                if ( bFlag3 & 0x20 ) != 0:
                    #print 'Spare bits set to zero'
                    pass

                #//I021/170 //Target Identification: 6 bytes
		#//ld new item
                if ( bFlag3 & 0x10 ) != 0:
                    cc = ReadCallCode( dFrame[Offset:] )
                    #print 'Target Identification %s' % cc
                    Offset += 6
                    pass
                
                #//I021/095 //Velocity Accuracy: 1 uint8
                if ( bFlag3 & 0x08 ) != 0:
                    #print 'Velocity Accuracy'
                    Offset += 1
                    pass

                #//I021/SPARE //Spare bits set to zero
                if ( bFlag3 & 0x04 ) != 0:
                    #print 'Spare bits set to zero'
                    pass

                #//I021/200 //Target Status: 1 uint8
                if ( bFlag3 & 0x02 ) != 0:
                    #print 'Target Status'
                    Offset += 1
                    pass

            if cFX > 3:
                #print 'cFX > 3'
                pass

            print time, '|', cc, '|', tAddr, '|', lat, '|', lon, '|', ga, '|', fl,'|', ia, '|', ta, '|', spd, '|', heading
            aLine = fData.readline()


