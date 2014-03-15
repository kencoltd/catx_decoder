#-*- coding: utf-8 -*-

import datetime
import struct
import sys
import os

def CountFx( frame ):
    #Bytes = struct.unpack('2s'*(len( frame ) / 2),frame)
    FX = 0
    #print frame
    while ( frame[FX] & 0x01) != 0:
        FX = FX + 1
    FX += 1
    return FX

def Trans21GPSPos( frame ):
    #print frame
    iData = frame[0]
    iData <<=  8
    iData |= frame[1]
    iData <<=  8
    iData |= frame[2]
    iData <<=  8
    iData |= frame[3]

    #//LSB = 180/power(2,25)
    fLatitude = iData * 180.0/33554432
    
    iData = frame[4]
    iData <<=  8
    iData |= frame[5]
    iData <<=  8
    iData |= frame[6]
    iData <<=  8
    iData |= frame[7]
    #//LSB = 180/power(2,25)
    fLongitude = iData*180.0/33554432
    
    return fLatitude, fLongitude

def ReadTargetAddress( frame ):
    uiData = 0
    uiData = frame[0]
    uiData<<= 8
    uiData |= frame[1]
    uiData<<= 8
    uiData |= frame[2]

    return uiData

def ReadModeCHei( frame ):
    #print frame[0], frame[1], ( frame[0] << 8 + frame[1] ), ( frame[0] << 8 + frame[1] ) * 7.62
    return ( ( frame[0] << 8 ) + frame[1] ) * 7.62

def ReadGpsHei( frame ):
    return ( ( frame[0] << 8 ) + frame[1] ) * 1.905

def ReadCallCode( frame ):
    return GetStringFrom6Char( frame )

def GetStringFrom6Char( frame ):
    #print frame
    chCallCode = ''
    c = frame[0]
    c >>= 2 #//bit 48-43 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = frame[0] << 4
    c |= frame[1] >> 4#//bit 42-37 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = frame[1]<<2;
    c |= frame[2]>>6#//bit 36-31 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = frame[2]#//bit 30-25 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = frame[3]
    c>>= 2#//bit 24-19 of 6 bytes
    chCallCode += chr(SixBits2OneChar(c&0x3f))

    c = frame[3]<<4
    c |= frame[4]>>4#//bit 18-13 of 6 bytes
    chCallCode += chr( SixBits2OneChar(c&0x3f) )

    c = frame[4]<<2
    c |= frame[5]>>6#//bit 12-7 of 6 bytes
    chCallCode += chr( SixBits2OneChar(c&0x3f) )

    c = frame[5]#//bit 6-1 of 6 bytes
    chCallCode += chr( SixBits2OneChar(c&0x3f) )

    return chCallCode.replace( ' ','' )

def SixBits2OneChar( value ):
    if ( value > 0 ) and ( value < 27 ):
            return (value+ord('A')-1)
    else:
            return value

def ReadAirSpeed( frame ):
    uData = ( frame[0] << 8 ) + frame[1]
    if ( uData & 0x8000 ) != 0:
        return (uData&0x7fff) * 0.32
    else:
        return uData * 0.11304


def ReadTrueAirSpeed( frame ):
    unData = ( frame[0] << 8 ) + frame[1]
    #///1knot NM/H
    return unData * 0.514444

def ReadGroundSpeed( frame ):
    unData = ( frame[0] << 8 ) + frame[1]
    fSpeed = unData * 0.11304
    unData = ( frame[2] << 8 ) + frame[3]
    fHeading = unData * 0.005493
    
    return fSpeed, fHeading

def ReadTime( frame ):
    uiData = 0
    uiData = frame[0]
    uiData <<= 8
    uiData |= frame[1]
    uiData <<= 8
    uiData |= frame[2]

    if uiData > 0:
        uiData -= 1

    hour = uiData / 128 / 3600
    minu = ( ( uiData / 128 ) % 3600 ) / 60
    sec = ( ( uiData / 128 ) % 3600 ) % 60
    minsec = ( uiData % 128 ) / 128.0
    
    return hour, minu, sec, minsec, uiData


def ReadSSR( frame ):
    if len( frame ) < 2:
        return '0000'
    unData = ( frame[0] << 8 ) + frame[1]
    '''
    //SSR 位16(V)  = 0，代码有效；    = 1，无效
    //    位15(G)  = 0，缺省；        = 1，交织应答
    //    位14(L)  = 0，从应答器获得； =  1，由本地跟踪器提供的平滑代码
    //    位13     备用位，置“0”
    //    位12-1   = 用八进制数表示的3/A模式回答代码
    '''
    if ( ( unData & 0x8000 ) == 0 ) or \
       ( ( unData & 0x4000 ) != 0 ) or \
       ( ( unData & 0x2000 ) )!= 0:
        return oct( ( unData & 0x0fff ) )
    else:
        return '0000'
    
def ReadOneFrame( frame ):                
    ga = 0.0
    ia = 0.0
    ta = 0.0
    spd = 0.0 
    heading = 0.0
    time = 0
    cc = ''
    tAddr = 0
    lat = 0.0
    lon = 0.0
    fl = 0.0
    SSR = ''

    if frame[0] != 21:
        print 'not cat21 data'
        return False
        #continue

    
    #if int( frame[1]+frame[2], 16 ) != len( frame ):
    
    if ( ( frame[1] << 8 ) + frame[2] ) != len( frame ):
        print 'not enough data'
        #aLine = fData.readline()
        return False
        #continue
    
    cFX = CountFx( frame[3:] )

    if cFX == 0:
        print 'no FX, '
        #aLine = fData.readline()
        return False
        #aCRC = fData.read(8)
        #aChar21 = fData.read(1)
        #continue
    else:
        #print 'FX, %d' % cFX
        pass
        
    Offset = 3 + cFX
    bFlag1 = frame[3]
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
        hour, minu, sec, minsec, rawdata = ReadTime( frame[Offset:] )

        dDate = sys.argv[1].split('/')
        dRecHour = int( dDate[3][0:2] )
        dActDay = int(dDate[1][6:8])

        if dRecHour == 0 and hour == 23:
            dActDay -= 1

        if hour == 24:
            hour = 0
        try:
            time = datetime.datetime(
            int(dDate[1][0:4]),
            int(dDate[1][4:6]),
            dActDay,
            hour,
            minu,
            sec,
            int(minsec * 1000000) )
        except ValueError:
            print hour, minu, sec, minsec, rawdata
            #sys.exit(0)
            pass
    #print time

        Offset += 3
        pass

    #//I021/130 //Position in WGS-84 Co-ordinates: 8 bytes
    if ( bFlag1 & 0x10 ) != 0:
        #print 'Position in WGS-84 Co-ordinates'
        lat, lon = Trans21GPSPos( frame[Offset:] )
        #print lat, lon
        Offset += 8
        pass

    #//I021/080 //Target Address: 3 bytes
    if ( bFlag1 & 0x08 ) != 0:
        tAddr = ReadTargetAddress( frame[Offset:] )
        #print 'Target Address 0x%X' % tAddr
        Offset += 3
        pass

    #//I021/140 //Geometric iAltitude: 2 bytes
    if ( bFlag1 & 0x04 ) != 0:
        ga = ReadGpsHei( frame[Offset:] )
        #print 'Geometric iAltitude %d' % ga
        Offset += 2
        pass
    
    #//I021/090 //Figure of Merit: 2 bytes
    if ( bFlag1 & 0x02 ) != 0:
        #print 'Figure of Merit'
        Offset += 2
        pass

    if cFX > 1:
        bFlag2 = frame[4]
        #print 'Flag 2, 0x%X' % bFlag2
        
        #//I021/210 //Link Technology: 1 uint8
        if ( bFlag2 & 0x80 ) != 0:
            #print 'Link Technology'
            Offset += 1
            pass
        
        #//I021/145 //Flight Level: 2 bytes
        if ( bFlag2 & 0x20 ) != 0:
            fl = ReadModeCHei( frame[Offset:] )
            #print 'Flight Level %d' % fl
            Offset += 2
            pass

        #//I021/150 //indicated Air Speed: 2 bytes
        if ( bFlag2 & 0x10 ) != 0:
            #print 'indicated Air Speed'
            #ia = ReadAirSpeed( frame[Offset:] )
            Offset += 2
            pass
        
        #//I021/151 //true Air Speed: 2 bytes
        if ( bFlag2 & 0x08 ) != 0:
            #print 'true Air Speed'
            #ta = ReadTrueAirSpeed( frame[Offset:] )
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
        bFlag3 = frame[5]
        #print 'Flag 3, 0x%X' % bFlag3
        
        #//I021/157 //Geometric Vertical Rate: 2 bytes
        if ( bFlag3 & 0x80 ) != 0:
            #print 'Geometric Vertical Rate'
            Offset += 2
            pass
        
        #//I021/160 //Ground Vector: 4 bytes
        if ( bFlag3 & 0x40 ) != 0:
            #print 'Ground Vector'
            spd, heading = ReadGroundSpeed( frame[Offset:] )
            Offset += 4
            pass

        #//I021/SPARE //Spare bits set to zero
        if ( bFlag3 & 0x20 ) != 0:
            #print 'Spare bits set to zero'
            pass

        #//I021/170 //Target Identification: 6 bytes
        #//ld new item
        if ( bFlag3 & 0x10 ) != 0:
            cc = ReadCallCode( frame[Offset:] )
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
        bFlag4 = frame[6]
        #//021/020 //Emitter Category: 1 uint8
        if ( bFlag4 & 0x80 ) != 0:
            #print 'Emitter Category'
            Offset += 1
            pass
        
        '''
        //021/spare
        //021/spare
        //021/spare
        //021/spare
        '''
        #//021/070 //Mode 3/A Code: 2 bytes
        if (bFlag4 & 0x04) != 0:
            SSR = ReadSSR( frame[Offset:] )
            Offset += 2
        
        #//021/131 Signal Amplitude 1BYTE
        if (bFlag4 & 0x02) != 0:
            #print 'Signal Amplitude'
            Offset += 1
        

        pass

    if ( lat >= 0.0 and lat <= 90.0 ) and \
       ( lon >= 0.0 and lon <= 180.0 ):# and \
       #( spd != 0.0 and heading != 0.0 ):
        print time, '|' ,cc, '|', SSR,'|', tAddr, '|', lat, '|', lon, '|', ga, '|', fl,'|', spd, '|', heading

    return True

if __name__ == '__main__':
    if len( sys.argv ) != 2:
        print 'input file name'
        sys.exit(0)

    if not os.path.exists( sys.argv[1] ):
        print 'input file not exist'
        sys.exit(0)
        
    fData = open( sys.argv[1], 'rb' )
    fData.read(26)
    aChar21 = fData.read(1)
    
    while aChar21 != '' :
        cLen = fData.read(2)
        if len( cLen ) != 2:
            break
        
        dLen = ( ord( cLen[0] ) << 8 ) + ord( cLen[1] )
        if dLen >= 256:
            print 'bad data len, %d' % dLen
            break
                                              
        bData = fData.read( dLen -3 )
        if len( bData ) != dLen - 3:
            print 'end of data, %d' % len( bData )
            break

        cFrame = aChar21 + cLen + bData
        dFrame = []
        for e in cFrame:
            dFrame.append( ord( e ) )

        #print dFrame
            
        if ReadOneFrame( dFrame ) != True:
            break

        aCRC = fData.read(8)
        aChar21 = fData.read(1)


