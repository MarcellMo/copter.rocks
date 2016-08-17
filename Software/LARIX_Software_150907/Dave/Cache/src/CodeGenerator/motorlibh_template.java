package CodeGenerator;

import java.util.*;
import com.ifx.davex.appjetinteract.App2JetInterface;

public class motorlibh_template
{
  protected static String nl;
  public static synchronized motorlibh_template create(String lineSeparator)
  {
    nl = lineSeparator;
    motorlibh_template result = new motorlibh_template();
    nl = null;
    return result;
  }

  public final String NL = nl == null ? (System.getProperties().getProperty("line.separator")) : nl;
  protected final String TEXT_1 = NL + "/******************************************************************************" + NL + " *" + NL + " * Copyright (C) 2011 Infineon Technologies AG. All rights reserved." + NL + " *" + NL + " * Infineon Technologies AG (Infineon) is supplying this software for use with" + NL + " * Infineon's microcontrollers." + NL + " * This file can be freely distributed within development tools that are" + NL + " * supporting such microcontrollers." + NL + " *" + NL + " * THIS SOFTWARE IS PROVIDED \"AS IS\".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED" + NL + " * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF" + NL + " * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE." + NL + " * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL," + NL + " * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER." + NL + " *" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "**                                                                            **" + NL + "** PLATFORM : Infineon XMC4000 Series /XMC1000 Series                         **" + NL + "**                                                                            **" + NL + "** COMPILER : Compiler Independent                                            **" + NL + "**                                                                            **" + NL + "** AUTHOR   : App Developer                                                   **" + NL + "**                                                                            **" + NL + "** MAY BE CHANGED BY USER [yes/no]: Yes                                       **" + NL + "**                                                                            **" + NL + "** MODIFICATION DATE : 27 Sept, 2013                                          **" + NL + "**                                                                            **" + NL + "*******************************************************************************/" + NL + "" + NL + "/*******************************************************************************" + NL + "**                       Author(s) Identity                                   **" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "** Initials     Name                                                          **" + NL + "** ---------------------------------------------------------------------------**" + NL + "** PA           App Developer                                                 **" + NL + "*******************************************************************************/" + NL + "/**" + NL + " * @file MOTORLIB.h" + NL + " *" + NL + " * @brief This file contains all public data structures,enums and function" + NL + " *        prototypes for motor control library." + NL + " *" + NL + " */" + NL + "/* Revision History" + NL + " * 03 Apr 2013   v1.0.0    Initial version" + NL + " * 25 Apr 2013   v1.0.2    Added support for XMC1302" + NL + " * 29 May 2013   v1.0.12   Added FOC support for XMC1302" + NL + " * 21 Jun 2013   v1.0.14   Added support for XMC4400 and XMC4200 device" + NL + " * 24 Jul 2013   v1.0.18   Added FOC support for XMC4400 device" + NL + " * 27 Sept 2013  v1.0.21   Added FOC support for XMC45 & XMC42 device" + NL + " */ " + NL + "#ifndef MOTORLIBS_H_" + NL + "#define MOTORLIBS_H_" + NL + "/*******************************************************************************" + NL + "**                      Include Files                                         **" + NL + "*******************************************************************************/" + NL + "#include <DAVE3.h>" + NL + "" + NL + "/**" + NL + "  * @ingroup MOTORLIBS_publicparam" + NL + "  * @{" + NL + "  */" + NL + "/*******************************************************************************" + NL + "**                      Global Macro Definitions                              **" + NL + "*******************************************************************************/";
  protected final String TEXT_2 = "    " + NL + "/*Limit1 is the higher limit and Limit2 is the lower limit and Buffer is the value which need to compare*/" + NL + "/**This is Clarke Transformation Scaling*/" + NL + "#define MOTORLIBS_CLARKESCALE  (0x01U)" + NL + "" + NL + "/**This is Inverse of Square root of 3 (1/sqrt(3)*2^15)*/" + NL + "#define MOTORLIBS_INVSQRT3 (0x49E6)" + NL + "" + NL + "/**This is Angle Scaling - Convert 24 bit to 16 bit*/" + NL + "#define MOTORLIBS_ANGLESCALE  (0x08U)" + NL + "" + NL + "/**This is Amplitude Scaling - Convert 24 bit to 16 bit*/" + NL + "#define MOTORLIBS_AMPSCALE  (0x08U)" + NL + "" + NL + "/**This is PI Controller KP Values SCaling*/" + NL + "#define MOTORLIBS_PI_KPSCALE  (0x09U)" + NL + "" + NL + "/**This is PI Controller KI Values SCaling*/" + NL + "#define MOTORLIBS_PI_KISCALE  (0x0FU)" + NL + "" + NL + "/*Limit1 is the higher limit and Limit2 is the lower limit and Buffer is the value which need to compare*/" + NL + "#define MOTORLIBS_MIN_MAX_LIMIT(Buffer,Limit1,Limit2) ((Buffer) > (Limit1)) ? (Limit1) : (((Buffer) < (Limit2))? (Limit2): (Buffer))";
  protected final String TEXT_3 = NL + "/**This is used to find out the absolute value*/" + NL + "#define MOTORLIBS_ABS(x) ((x > 0) ? x : -x)" + NL + "" + NL + "/**This is CORIDC K values used in SW CORIDC*/" + NL + "#define CORDIC_K \t\t\t1304065792U" + NL + "" + NL + "/**This is CORIDC Iteration, maximum values is 23*/" + NL + "#define CORDIC_ITERATIONS\t15" + NL + "" + NL + "/**" + NL + " * This enumerates magnitude prescaler value for calculated value of X & Y" + NL + " */" + NL + "typedef enum" + NL + "{" + NL + "  /**" + NL + "   * After the last iteration of calculation the X & Y values are divided by 1" + NL + "   */" + NL + "  MOTORLIBS_MPS_DIVBY1 = 0x00U," + NL + "  /**" + NL + "   * After the last iteration of calculation the X & Y values are divided by 2" + NL + "   */" + NL + "  MOTORLIBS_MPS_DIVBY2 = 0x01U," + NL + "  /**" + NL + "   * After the last iteration of calculation the X & Y values are divided by 4" + NL + "   */" + NL + "  MOTORLIBS_MPS_DIVBY4 = 0x02U" + NL + "}MOTORLIBS_CON_MPS;" + NL;
  protected final String TEXT_4 = NL + "/*******************************************************************************" + NL + " *                                ENUMERATIONS                                **" + NL + " ******************************************************************************/";
  protected final String TEXT_5 = NL + "/**" + NL + " * This enumerates control to Keep or Clear the last X result as initial data" + NL + " * for next calculation" + NL + " */" + NL + "typedef enum MOTORLIBS_STATC_KEEPX" + NL + "{" + NL + "  /**" + NL + "   * This clears last X Result for a New Calculation" + NL + "   */" + NL + "  MOTORLIBS_CLEAR_KEEPX = 0x00U," + NL + "  /**" + NL + "   * This keeps last X Result as Initial Data for a New Calculation" + NL + "   */" + NL + "  MOTORLIBS_SET_KEEPX = 0x20U" + NL + "} MOTORLIBS_STATC_KEEPX;" + NL + "/**" + NL + " * This enumerates control to Keep or Clear the last Y result as initial data" + NL + " * for next calculation" + NL + " */" + NL + "typedef enum MOTORLIBS_STATC_KEEPY" + NL + "{" + NL + "  /**" + NL + "   * This clears last Y Result for a New Calculation" + NL + "   */" + NL + "  MOTORLIBS_CLEAR_KEEPY = 0x00U," + NL + "  /**" + NL + "   * This keeps last Y Result as Initial Data for a New Calculation" + NL + "   */" + NL + "  MOTORLIBS_SET_KEEPY = 0x40U" + NL + "} MOTORLIBS_STATC_KEEPY;" + NL + "" + NL + "/**" + NL + " * This enumerates control to Keep or Clear the last Z result as initial data" + NL + " * for next calculation" + NL + " */" + NL + "typedef enum MOTORLIBS_STATC_KEEPZ" + NL + "{" + NL + "  /**" + NL + "   * This clears last Z Result for a New Calculation" + NL + "   */" + NL + "  MOTORLIBS_CLEAR_KEEPZ = 0x00U," + NL + "  /**" + NL + "   * This keeps last Z Result as Initial Data for a New Calculation" + NL + "   */" + NL + "  MOTORLIBS_SET_KEEPZ = 0x80U" + NL + "} MOTORLIBS_STATC_KEEPZ;" + NL + "/**" + NL + " * This enumerates different modes for CORDIC operating modes" + NL + " */" + NL + "typedef enum MOTORLIBS_CON_MODE" + NL + "{ " + NL + "  /**" + NL + "   * This sets CORDIC to Linear Operating Mode" + NL + "   */" + NL + "  MOTORLIBS_LINEAR_MODE = 0x0U," + NL + "  /**" + NL + "   * This sets CORDIC to Circular Operating Mode" + NL + "   */" + NL + "  MOTORLIBS_CIRCULAR_MODE = 0x2U," + NL + "  /**" + NL + "   * This sets CORDIC to Hyperbolic Operating Mode" + NL + "   */" + NL + "  MOTORLIBS_HYPERBOLIC_MODE = 0x6U" + NL + "} MOTORLIBS_CON_MODE;" + NL + "" + NL + "/**" + NL + " * This enumerates CORDIC Rotation Vectoring Selection" + NL + " */" + NL + "typedef enum MOTORLIBS_CON_ROTVEC" + NL + "{" + NL + "  /**" + NL + "   * This sets CORDIC to Vectoring Mode" + NL + "   */" + NL + "  MOTORLIBS_VECTORING_MODE = 0x0U," + NL + "  /**" + NL + "   * This sets CORDIC to Rotation Mode" + NL + "   */" + NL + "  MOTORLIBS_ROTATION_MODE = 0x8U" + NL + "} MOTORLIBS_CON_ROTVEC;" + NL + "" + NL + "/**" + NL + " * This enumerates different modes for starting CORDIC operation" + NL + " */" + NL + "typedef enum MOTORLIBS_CON_STMODE" + NL + "{" + NL + "  /**" + NL + "   * Auto start of calculation after write access to X parameter data register" + NL + "   */" + NL + "  MOTORLIBS_CORDIC_AUTO_START = 0x00U," + NL + "  /**" + NL + "   * Start calculation only after bit ST is set" + NL + "   */" + NL + "  MOTORLIBS_CORDIC_ST_START = 0x10U" + NL + "} MOTORLIBS_CON_STMODE;" + NL + "" + NL + "/**" + NL + " * This enumerates 'X result register' data format in circular vectoring mode." + NL + " */" + NL + "typedef enum MOTORLIBS_CON_XUSIGN" + NL + "{" + NL + "  /**" + NL + "   * This sets X result data format to Signed (twos complement), when read" + NL + "   */" + NL + "  MOTORLIBS_SIGNED_X_RESULT = 0x00U," + NL + "  /**" + NL + "   * This sets X result data format to Unsigned, when read" + NL + "   */" + NL + "  MOTORLIBS_UNSIGNED_X_RESULT = 0x20U" + NL + "}MOTORLIBS_CON_XUSIGN;" + NL + "" + NL + "/**" + NL + " * This enumerates magnitude prescaler value for calculated value of X & Y" + NL + " */" + NL + "typedef enum MOTORLIBS_CON_MPS" + NL + "{" + NL + "  /**" + NL + "   * After the last iteration of calculation the X & Y values are divided by 1" + NL + "   */" + NL + "  MOTORLIBS_MPS_DIVBY1 = 0x00U," + NL + "  /**" + NL + "   * After the last iteration of calculation the X & Y values are divided by 2" + NL + "   */" + NL + "  MOTORLIBS_MPS_DIVBY2 = 0x40U," + NL + "  /**" + NL + "   * After the last iteration of calculation the X & Y values are divided by 4" + NL + "   */" + NL + "  MOTORLIBS_MPS_DIVBY4 = 0x80U" + NL + "}MOTORLIBS_CON_MPS;" + NL;
  protected final String TEXT_6 = NL + NL + "/*******************************************************************************" + NL + " *                             STRUCTURES                                     **" + NL + " ******************************************************************************/" + NL + "/**" + NL + " * This structure holds the values of PI parameters." + NL + " */" + NL + "typedef struct MOTORLIBS_PIHandleType" + NL + "{" + NL + "  /**" + NL + "   * This is the Kp value" + NL + "   */" + NL + "  uint16_t  Kp;  " + NL + " /**" + NL + "  * This is the Ki value" + NL + "  */" + NL + "  uint16_t  Ki; " + NL + " /**" + NL + "  * This is the buffer" + NL + "  */" + NL + "  int32_t  Ibuf; " + NL + "  /**" + NL + "   * Maximum limit for buffer" + NL + "   */" + NL + "  const int32_t  Yimax;    " + NL + "  /**" + NL + "   * Minimum limit for buffer" + NL + "   */" + NL + "  const int32_t  Yimin; " + NL + "  /**" + NL + "   * Maximum limit for PI controller output" + NL + "   */" + NL + "  const int16_t  Ymax;     " + NL + "  /**" + NL + "   * Minimum limit for PI controller output" + NL + "   */" + NL + "  const int16_t  Ymin; " + NL + "  /**" + NL + "   * This is the required output value for PI controller" + NL + "   */  " + NL + "  int16_t PiOutVal;" + NL + "  " + NL + "} MOTORLIBS_PIHandleType;" + NL + " " + NL + "/**" + NL + " * This structure holds the values of PT1 filter parameters." + NL + " */" + NL + "typedef struct MOTORLIBS_PT1HandleType" + NL + "{" + NL + "  /**" + NL + "   * Filter constant " + NL + "   */" + NL + "  int32_t  Z1;" + NL + "  /**" + NL + "   * Filter constant " + NL + "   */" + NL + "  int32_t  Z2;" + NL + "  /**" + NL + "   * Maximum limit of PT1 buffer" + NL + "   */" + NL + "  const int32_t  Ymax;" + NL + "  /**" + NL + "   * Minimum limit of PT1 buffer" + NL + "   */" + NL + "  const int32_t  Ymin;" + NL + "  /**" + NL + "   * This is the integral buffer of pt1 filter" + NL + "   */" + NL + "  int32_t  PT1buf;" + NL + "  /**" + NL + "   * This is PT1 filter output." + NL + "   */" + NL + "  int32_t  PT1OutVal;" + NL + "  " + NL + "} MOTORLIBS_PT1HandleType;" + NL + " " + NL + "/**" + NL + " * @}" + NL + " */" + NL + "/**" + NL + " * @ingroup MOTORLIBS_apidoc" + NL + " * @{" + NL + " */" + NL + "/*******************************************************************************" + NL + " **FUNCTION PROTOTYPES                                                        **" + NL + "*******************************************************************************/" + NL + "/**" + NL + "  * @brief This function is the implementation of PI controller. " + NL + "  * PI Output = kp*error + Ki * integral of error" + NL + "  * @param[in] HandlePtr Pointer to MOTORLIBS_PIHandleType" + NL + "  * @param[in] RefValue int32_t which is reference value for PI controller" + NL + "  * @param[in] ActValue int32_t which is actual value for PI controller" + NL + "  * " + NL + "  * @return   void <BR>" + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "void MOTORLIBS_PIController" + NL + "(" + NL + "    MOTORLIBS_PIHandleType* HandlePtr," + NL + "    int32_t RefValue," + NL + "    int32_t ActValue" + NL + ");" + NL + "/**" + NL + "  * @brief This function is the implementation of PT1 filter. " + NL + "  * Yn[3,2,1,0]  =Yn-1[3,2,1,0] +Z1*(Xn -Yn-1[3,2])" + NL + "  * @param[in] HandlePtr PT1 Handle of the MOTORLIBS App" + NL + "  * @param[in] ActValue int32_t which is actual value for PT1 filter" + NL + "  * " + NL + "  * @return  void <BR>" + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "void MOTORLIBS_PT1Controller" + NL + "(" + NL + "   MOTORLIBS_PT1HandleType* HandlePtr," + NL + "   int32_t ActValue" + NL + ");" + NL + "/**" + NL + "  * @brief This function is the implementation of PT12 filter. " + NL + "  * Yn[3,2,1,0]  =Yn-1[3,2,1,0] +(Z1*Xn -Z2*Yn-1[3,2])" + NL + "  * @param[in] HandlePtr PT1 Handle of the MOTORLIBS App" + NL + "  * @param[in] ActValue int32_t which is actual value for PT1 filter" + NL + "  * " + NL + "  * @return  void <BR>" + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "void MOTORLIBS_PT12Controller" + NL + "(" + NL + "   MOTORLIBS_PT1HandleType* HandlePtr," + NL + "   int32_t ActValue" + NL + ");" + NL + "" + NL + "/**" + NL + "  * @brief This function is the implementation of clark transform." + NL + "  * Ialpha = Phase_A/2" + NL + "  * Ibeta  = (Phase_A+2*Phase_B)/(2*Sqrt(3))" + NL + "  * @param[in] Phase_A  int32_t" + NL + "  * @param[in] Phase_B  int32_t" + NL + "  * @param[in] Ialpha pointer to int32_t." + NL + "  * " + NL + "  * @return  int32_t <BR>" + NL + "  *         " + NL + "  *         " + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "int32_t MOTORLIBS_ClarkTransform(int32_t Phase_A, int32_t Phase_B, int32_t *Ialpha );" + NL + "/**" + NL + "  * @brief This function is the implementation of clark transform." + NL + "  * Ialpha = Phase_A/2" + NL + "  * Ibeta  = (Phase_B-Phase_C)/(2*Sqrt(3))" + NL + "  * @param[in] Phase_A  int32_t" + NL + "  * @param[in] Phase_B  int32_t" + NL + "  * @param[in] Phase_C  int32_t" + NL + "  * @param[in] Ialpha pointer to int32_t." + NL + "  * " + NL + "  * @return  int32_t <BR>" + NL + "  *         " + NL + "  *         " + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "int32_t MOTORLIBS_ClarkTransform2(int32_t Phase_A, int32_t Phase_B, int32_t Phase_C, int32_t *Ialpha );" + NL + "/**" + NL + "  * @brief This function is the implementation of park transform. " + NL + "  * Id  =k*(Ialpha*cos(Anlge)+Ibeta*sin(Angle))/2  ; where k = 1.64767" + NL + "  * Iq  =k*(-Ialpha*sin(Anlge)+Ibeta*cos(Angle))/2 ; where k = 1.64767" + NL + "  * @param[in] Ialpha int32_t" + NL + "  * @param[in] Ibeta  int32_t" + NL + "  * @param[in] Angle  int16_t" + NL + "  * @param[in] Iq pointer to int32_t." + NL + "  * " + NL + "  * @return  int32_t <BR>" + NL + "  *         " + NL + "  *         " + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "int32_t MOTORLIBS_ParkTransform(int32_t Ialpha,int32_t Ibeta,int16_t Angle,int32_t *Iq);" + NL + "" + NL + "/**" + NL + "  * @brief This function is the implementation of inverse park transform. " + NL + "  * Valpha  = k*(Vd*cos(Anlge)- Vq*sin(Angle))/4 ; where k = 1.64767" + NL + "  * Vbeta   = k*(Vd*sin(Anlge)+Vq*cos(Angle))/4 ; where k = 1.64767" + NL + "  * @param[in] Vd  int32_t" + NL + "  * @param[in] Vq  int32_t" + NL + "  * @param[in] Angle int16_t" + NL + "  * @param[in] Vbeta pointer to int32_t." + NL + "  * " + NL + "  * @return  int32_t <BR>" + NL + "  *         " + NL + "  *         " + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "" + NL + "int32_t MOTORLIBS_IParkTransform(int32_t Vd,int32_t Vq,int16_t Angle,int32_t *Vbeta);" + NL + "" + NL + "/**" + NL + "  * @brief This function is the implementation cartesian to polar transform. " + NL + "  * Amlpitude  = K * SQRT(x*x+y*y); where k = 1.64767" + NL + "  * Angle      = ATAN(y/x);" + NL + "  * @param[in] x  int32_t" + NL + "  * @param[in] y  int32_t" + NL + "  * @param[in] Angle pointer to int16_t." + NL + "  * " + NL + "  * @return  uint32_t <BR>" + NL + "  *         " + NL + "  *         " + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "uint32_t MOTORLIBS_Car2Pol(int32_t x,int32_t y, int16_t *Angle);" + NL + "/**" + NL + "  * @brief This function is the implementation polar to cartesian transform. " + NL + "  * X = K * Amplitude* cos(Angle); where k = 1.64767" + NL + "  * Y = K * Amplitude* sin(Angle); where k = 1.64767" + NL + "  * @param[in] Angle  int16_t" + NL + "  * @param[in] Amplitude  uint32_t" + NL + "  * @param[in] X pointer to int32_t holding the x co-ordinate value." + NL + "  * " + NL + "  * @return  int32_t <BR>" + NL + "  *         " + NL + "  *         " + NL + "  * <b>Reentrancy:  Yes</b><BR>" + NL + "  * " + NL + "  * <b>Sync/Async:  Synchronous</b><BR>" + NL + "  */" + NL + "int32_t MOTORLIBS_Pol2Car(int16_t Angle,uint32_t Amplitude, int32_t *X);" + NL + "" + NL + "/**" + NL + " * @}" + NL + " */" + NL + "#endif /* MOTORLIBS_H_ */" + NL + NL + NL + NL;
  protected final String TEXT_7 = NL;

  public String generate(Object argument)
  {
    final StringBuffer stringBuffer = new StringBuffer();
     App2JetInterface app = (App2JetInterface) argument; 
    stringBuffer.append(TEXT_1);
     String AppBaseuri = "app/motorlibs/"; 
     int Is44Device = -1; 
     int Is42Device = -1; 
     int Is4xDevice = -1; 
     int Is13Device = -1; 
     Is4xDevice = ((app.getSoftwareId().substring(0,1).compareTo("4")==0)?1:0); 
     Is44Device = ((app.getSoftwareId().substring(0,2).compareTo("44")==0)?1:0); 
     Is42Device = ((app.getSoftwareId().substring(0,2).compareTo("42")==0)?1:0); 
     Is13Device = ((app.getSoftwareId().substring(0,2).compareTo("13")==0)?1:0); 
    stringBuffer.append(TEXT_2);
    if(Is4xDevice == 1){
    stringBuffer.append(TEXT_3);
    }
    stringBuffer.append(TEXT_4);
    if(Is13Device == 1){
    stringBuffer.append(TEXT_5);
    }
    stringBuffer.append(TEXT_6);
    stringBuffer.append(TEXT_7);
    return stringBuffer.toString();
  }
}