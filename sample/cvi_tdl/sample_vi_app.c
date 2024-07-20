#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/param.h>
#include <sys/prctl.h>
#include <inttypes.h>

#include "cvi_buffer.h"
#include "cvi_ae_comm.h"
#include "cvi_awb_comm.h"
#include "cvi_comm_isp.h"

#include "sample_comm.h"

#include "middleware_utils.h"
#include "sample_utils.h"
#include "vi_vo_utils.h"

#include <core/utils/vpss_helper.h>
#include <cvi_comm.h>
#include <rtsp.h>
#include <sample_comm.h>
#include "cvi_tdl.h"

// thank lxowalle

#define NONE "\033[m"
#define RED "\033[0;32;31m"
#define GREEN "\033[0;32;32m"

#define PACKED_32BIT(a, b, c, d) ((a << 24) + (b << 16) + (c << 8) + d)
#define PACKED_16BIT(c, d) ((c << 8) + d)

#define PR2100_WIDTH 1920
#define PR2100_HEIGHT 1080
#define PR2100_CHID_SIZE 2
#define PR2100_LINE_VALID(a) (((a >> 11) & 1) ? 1 : 0)
#define PR2100_LINE_NUMBER(a) (a & 0x7ff)
#define PR2100_FRAME_NUMBER(a) ((a >> 12) & 0x7)
#define SENSOR0_TYPE GCORE_GC4653_MIPI_4M_30FPS_10BIT
typedef struct _SAMPLE_VPSS_PARAM_S
{
    VPSS_GRP SrcGrp;
    VPSS_CHN SrcChn;
    VPSS_GRP DstGrp;
    VPSS_CHN DstChn;
    RECT_S stDispRect;
} SAMPLE_VPSS_PARAM_S;

typedef struct _SAMPLE_VPSS_CONFIG_S
{
    CVI_S32 s32ChnNum;
    CVI_BOOL stop_thread;
    pthread_t vpss_thread;
    SAMPLE_VPSS_PARAM_S astVpssParam[4];
} SAMPLE_VPSS_CONFIG_S;

typedef struct
{
    SAMPLE_TDL_MW_CONTEXT *pstMWContext;
    cvitdl_service_handle_t stServiceHandle;
} SAMPLE_TDL_VENC_THREAD_ARG_S;

SAMPLE_VO_CONFIG_S g_stVoConfig;

static volatile bool bExit = false;

static cvtdl_face_t g_stFaceMeta = {0};

static uint32_t g_size = 0;

MUTEXAUTOLOCK_INIT(ResultMutex);

CVI_S32 SAMPLE_PLAT_VO_INIT2(void)
{
    SAMPLE_VO_CONFIG_S stVoConfig;
    RECT_S stDefDispRect = {0, 0, 368, 552};
    SIZE_S stDefImageSize = {368, 552};
    // RECT_S stDefDispRect  = {0, 0, 480, 640};
    // SIZE_S stDefImageSize = {480, 640};
    CVI_S32 s32Ret = CVI_SUCCESS;

    CVI_U32 panel_init = false;
    VO_PUB_ATTR_S stVoPubAttr;

    CVI_VO_Get_Panel_Status(0, 0, &panel_init);
    if (panel_init)
    {
        CVI_VO_GetPubAttr(0, &stVoPubAttr);
        CVI_TRACE_LOG(CVI_DBG_NOTICE, "Panel w=%d, h=%d.\n",
                      stVoPubAttr.stSyncInfo.u16Hact, stVoPubAttr.stSyncInfo.u16Vact);
        stDefDispRect.u32Width = stVoPubAttr.stSyncInfo.u16Hact;
        stDefDispRect.u32Height = stVoPubAttr.stSyncInfo.u16Vact;
        stDefImageSize.u32Width = stVoPubAttr.stSyncInfo.u16Hact;
        stDefImageSize.u32Height = stVoPubAttr.stSyncInfo.u16Vact;
    }

    s32Ret = SAMPLE_COMM_VO_GetDefConfig(&stVoConfig);
    if (s32Ret != CVI_SUCCESS)
    {
        CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_VO_GetDefConfig failed with %#x\n", s32Ret);
        goto error;
    }

    stVoConfig.VoDev = 0;
    stVoConfig.stVoPubAttr.enIntfType = VO_INTF_MIPI;
    stVoConfig.stVoPubAttr.enIntfSync = VO_OUTPUT_720x1280_60;
    stVoConfig.stDispRect = stDefDispRect;
    stVoConfig.stImageSize = stDefImageSize;
    stVoConfig.enPixFormat = SAMPLE_PIXEL_FORMAT;
    stVoConfig.enVoMode = VO_MODE_1MUX;

    memcpy(&g_stVoConfig, &stVoConfig, sizeof(SAMPLE_VO_CONFIG_S));
    s32Ret = SAMPLE_COMM_VO_StartVO(&stVoConfig);
    if (s32Ret != CVI_SUCCESS)
    {
        SAMPLE_PRT("SAMPLE_COMM_VO_StartVO failed with %#x\n", s32Ret);
        goto error;
    }

    return s32Ret;
error:
    // _SAMPLE_PLAT_ERR_Exit();
    return s32Ret;
}

CVI_S32 SAMPLE_PLAT_VO_DEINIT2(void)
{
    CVI_S32 s32Ret = CVI_SUCCESS;
    SAMPLE_COMM_VO_StopVO(&g_stVoConfig);
    return s32Ret;
}

CVI_S32 SAMPLE_VIO_TWO_DEV_VO(void)
{
    MMF_VERSION_S stVersion;
    SAMPLE_INI_CFG_S stIniCfg = {0};
    SAMPLE_VI_CONFIG_S stViConfig;

    PIC_SIZE_E enPicSize;
    // CVI_U32 chnID = 0;
    SIZE_S stSize;
    CVI_S32 s32Ret = CVI_SUCCESS;
    LOG_LEVEL_CONF_S log_conf;

    stIniCfg = (SAMPLE_INI_CFG_S){
        .enSource = VI_PIPE_FRAME_SOURCE_DEV,
        .devNum = 2,
        .enSnsType[0] = SONY_IMX327_2L_MIPI_2M_30FPS_12BIT,
        .enWDRMode[0] = WDR_MODE_NONE,
        .s32BusId[0] = 3,
        .MipiDev[0] = 0xff,
        .enSnsType[1] = SONY_IMX327_SLAVE_MIPI_2M_30FPS_12BIT,
        .s32BusId[1] = 0,
        .MipiDev[1] = 0xff,
    };

    CVI_SYS_GetVersion(&stVersion);
    SAMPLE_PRT("MMF Version:%s\n", stVersion.version);

    log_conf.enModId = CVI_ID_LOG;
    log_conf.s32Level = CVI_DBG_INFO;
    CVI_LOG_SetLevelConf(&log_conf);

    // Get config from ini if found.
    if (SAMPLE_COMM_VI_ParseIni(&stIniCfg))
    {
        SAMPLE_PRT("Parse complete\n");
    }

    // Set sensor number
    CVI_VI_SetDevNum(stIniCfg.devNum);

    /************************************************
     * step1:  Config VI
     ************************************************/
    s32Ret = SAMPLE_COMM_VI_IniToViCfg(&stIniCfg, &stViConfig);
    if (s32Ret != CVI_SUCCESS)
        return s32Ret;

    /************************************************
     * step2:  Get input size
     ************************************************/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stIniCfg.enSnsType[0], &enPicSize);
    if (s32Ret != CVI_SUCCESS)
    {
        CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_VI_GetSizeBySensor failed with %#x\n", s32Ret);
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSize);
    if (s32Ret != CVI_SUCCESS)
    {
        CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_SYS_GetPicSize failed with %#x\n", s32Ret);
        return s32Ret;
    }

    /************************************************
     * step3:  Init modules
     ************************************************/
    s32Ret = SAMPLE_PLAT_SYS_INIT(stSize);
    if (s32Ret != CVI_SUCCESS)
    {
        CVI_TRACE_LOG(CVI_DBG_ERR, "sys init failed. s32Ret: 0x%x !\n", s32Ret);
        return s32Ret;
    }

    s32Ret = SAMPLE_PLAT_VI_INIT(&stViConfig);
    if (s32Ret != CVI_SUCCESS)
    {
        CVI_TRACE_LOG(CVI_DBG_ERR, "vi init failed. s32Ret: 0x%x !\n", s32Ret);
        return s32Ret;
    }

    SIZE_S stSizeIn, stSizeOut;

    stSizeIn.u32Width = stSize.u32Width;
    stSizeIn.u32Height = stSize.u32Height;
    stSizeOut.u32Width = 552;
    stSizeOut.u32Height = 368;

    s32Ret = SAMPLE_PLAT_VPSS_INIT(0, stSizeIn, stSizeOut);
    if (s32Ret != CVI_SUCCESS)
    {
        SAMPLE_PRT("vpss init failed. s32Ret: 0x%x !\n", s32Ret);
        CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};
        abChnEnable[0] = CVI_TRUE;
        SAMPLE_COMM_VPSS_Stop(0, abChnEnable);
        s32Ret = SAMPLE_PLAT_VPSS_INIT(0, stSizeIn, stSizeOut);
        if (s32Ret != CVI_SUCCESS)
        {
            SAMPLE_PRT("vpss init failed. s32Ret: 0x%x !\n", s32Ret);
            return s32Ret;
        }
    }

    s32Ret = SAMPLE_PLAT_VPSS_INIT(1, stSizeIn, stSizeOut);
    if (s32Ret != CVI_SUCCESS)
    {
        SAMPLE_PRT("vpss init failed. s32Ret: 0x%x !\n", s32Ret);
        CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};
        abChnEnable[0] = CVI_TRUE;
        SAMPLE_COMM_VPSS_Stop(1, abChnEnable);
        s32Ret = SAMPLE_PLAT_VPSS_INIT(1, stSizeIn, stSizeOut);
        if (s32Ret != CVI_SUCCESS)
        {
            SAMPLE_PRT("vpss init failed. s32Ret: 0x%x !\n", s32Ret);
            return s32Ret;
        }
    }

    s32Ret = SAMPLE_COMM_VI_Bind_VPSS(0, 0, 0);
    if (s32Ret != CVI_SUCCESS)
    {
        SAMPLE_PRT("vi bind vpss failed. s32Ret: 0x%x !\n", s32Ret);
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_VI_Bind_VPSS(0, 1, 1);
    if (s32Ret != CVI_SUCCESS)
    {
        SAMPLE_PRT("vi bind vpss failed. s32Ret: 0x%x !\n", s32Ret);
        return s32Ret;
    }

    s32Ret = SAMPLE_PLAT_VO_INIT2();
    if (s32Ret != CVI_SUCCESS)
    {
        SAMPLE_PRT("vo init failed. s32Ret: 0x%x !\n", s32Ret);
        return s32Ret;
    }

    CVI_VO_SetChnRotation(0, 0, ROTATION_90);

    // CVI_S32 vo_bind_vpssgrp = 0;

    SAMPLE_COMM_VPSS_Bind_VO(0, 0, 0, 0);
    sleep(5);
    SAMPLE_COMM_VPSS_UnBind_VO(0, 0, 0, 0);
    SAMPLE_PLAT_VO_DEINIT2();

    {
        CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};
        abChnEnable[0] = CVI_TRUE;
        SAMPLE_COMM_VPSS_Stop(0, abChnEnable);
        SAMPLE_COMM_VPSS_Stop(1, abChnEnable);
    }

    SAMPLE_COMM_VI_DestroyIsp(&stViConfig);

    SAMPLE_COMM_VI_DestroyVi(&stViConfig);

    SAMPLE_COMM_SYS_Exit();

    return s32Ret;
}

void *run_venc(void *args)
{
    printf("Enter encoder thread\n");
    SAMPLE_TDL_VENC_THREAD_ARG_S *pstArgs = (SAMPLE_TDL_VENC_THREAD_ARG_S *)args;
    VIDEO_FRAME_INFO_S stFrame;
    CVI_S32 s32Ret;
    cvtdl_face_t stFaceMeta = {0};

    while (bExit == false)
    {
        s32Ret = CVI_VPSS_GetChnFrame(0, 0, &stFrame, 2000);
        if (s32Ret != CVI_SUCCESS)
        {
            printf("CVI_VPSS_GetChnFrame chn0 failed with %#x\n", s32Ret);
            break;
        }

        {
            MutexAutoLock(ResultMutex, lock);
            memset(&stFaceMeta, 0, sizeof(cvtdl_face_t));
            if (NULL != g_stFaceMeta.info)
            {
                CVI_TDL_CopyFaceMeta(&g_stFaceMeta, &stFaceMeta);
            }
            CVI_TDL_Free(&g_stFaceMeta);
        }

        s32Ret = CVI_TDL_Service_FaceDrawRect(pstArgs->stServiceHandle, &stFaceMeta, &stFrame, false,
                                              CVI_TDL_Service_GetDefaultBrush());
        if (s32Ret != CVI_TDL_SUCCESS)
        {
            CVI_VPSS_ReleaseChnFrame(0, 0, &stFrame);
            printf("Draw fame fail!, ret=%x\n", s32Ret);
            goto error;
        }

        s32Ret = SAMPLE_TDL_Send_Frame_RTSP(&stFrame, pstArgs->pstMWContext);
        if (s32Ret != CVI_SUCCESS)
        {
            CVI_VPSS_ReleaseChnFrame(0, 0, &stFrame);
            printf("Send Output Frame NG, ret=%x\n", s32Ret);
            goto error;
        }

    error:
        CVI_TDL_Free(&stFaceMeta);
        CVI_VPSS_ReleaseChnFrame(0, 0, &stFrame);
        if (s32Ret != CVI_SUCCESS)
        {
            bExit = true;
        }
    }
    printf("Exit encoder thread\n");
    pthread_exit(NULL);
}

void *run_tdl_thread(void *pHandle)
{
    printf("Enter TDL thread\n");
    cvitdl_handle_t pstTDLHandle = (cvitdl_handle_t)pHandle;

    VIDEO_FRAME_INFO_S stFrame;
    cvtdl_face_t stFaceMeta = {0};

    CVI_S32 s32Ret;
    while (bExit == false)
    {
        s32Ret = CVI_VPSS_GetChnFrame(0, VPSS_CHN1, &stFrame, 2000);

        if (s32Ret != CVI_SUCCESS)
        {
            printf("CVI_VPSS_GetChnFrame failed with %#x\n", s32Ret);
            goto get_frame_failed;
        }

        memset(&stFaceMeta, 0, sizeof(cvtdl_face_t));
        s32Ret = CVI_TDL_ScrFDFace(pstTDLHandle, &stFrame, &stFaceMeta);
        if (s32Ret != CVI_TDL_SUCCESS)
        {
            printf("inference failed!, ret=%x\n", s32Ret);
            goto inf_error;
        }

        if (stFaceMeta.size != g_size)
        {
            printf("face count: %d\n", stFaceMeta.size);
            g_size = stFaceMeta.size;
        }

        {
            MutexAutoLock(ResultMutex, lock);
            memset(&g_stFaceMeta, 0, sizeof(cvtdl_face_t));
            if (NULL != stFaceMeta.info)
            {
                CVI_TDL_CopyFaceMeta(&stFaceMeta, &g_stFaceMeta);
            }
        }

    inf_error:
        CVI_VPSS_ReleaseChnFrame(0, 1, &stFrame);
    get_frame_failed:
        CVI_TDL_Free(&stFaceMeta);
        if (s32Ret != CVI_SUCCESS)
        {
            bExit = true;
        }
    }

    printf("Exit TDL thread\n");
    pthread_exit(NULL);
}

static void SampleHandleSig(CVI_S32 signo)
{
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);
    printf("handle signal, signo: %d\n", signo);
    if (SIGINT == signo || SIGTERM == signo)
    {
        bExit = true;
    }
}
int main(int argc, char *argv[])
{
    CVI_S32 s32Ret = CVI_FAILURE;
    if (argc != 2)
    {
        printf(
            "\nUsage: %s RETINA_MODEL_PATH.\n\n"
            "\tRETINA_MODEL_PATH, path to retinaface model.\n",
            argv[0]);
        return CVI_TDL_FAILURE;
    }
    signal(SIGINT, SampleHandleSig);
    signal(SIGTERM, SampleHandleSig);

    // SAMPLE_VIO_TWO_DEV_VO();

    SAMPLE_TDL_MW_CONFIG_S stMWConfig = {0};

    s32Ret = SAMPLE_TDL_Get_VI_Config(&stMWConfig.stViConfig);
    if (s32Ret != CVI_SUCCESS || stMWConfig.stViConfig.s32WorkingViNum <= 0)
    {
        printf("Failed to get senor infomation from ini file (/mnt/data/sensor_cfg.ini).\n");
        return -1;
    }

    // Get VI size
    PIC_SIZE_E enPicSize;
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stMWConfig.stViConfig.astViInfo[0].stSnsInfo.enSnsType,
                                            &enPicSize);
    if (s32Ret != CVI_SUCCESS)
    {
        printf("Cannot get senor size\n");
        return -1;
    }

    SIZE_S stSensorSize;
    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSensorSize);
    if (s32Ret != CVI_SUCCESS)
    {
        printf("Cannot get senor size\n");
        return -1;
    }

    // Setup frame size of video encoder to 1080p
    SIZE_S stVencSize = {
        .u32Width = 1280,
        .u32Height = 720,
    };

    stMWConfig.stVBPoolConfig.u32VBPoolCount = 3;

    // VBPool 0 for VPSS Grp0 Chn0
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].enFormat = VI_PIXEL_FORMAT;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32BlkCount = 5;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32Height = stSensorSize.u32Height;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32Width = stSensorSize.u32Width;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].bBind = true;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32VpssChnBinding = VPSS_CHN0;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32VpssGrpBinding = (VPSS_GRP)0;

    // VBPool 1 for VPSS Grp0 Chn1
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].enFormat = VI_PIXEL_FORMAT;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32BlkCount = 5;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32Height = stVencSize.u32Height;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32Width = stVencSize.u32Width;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].bBind = true;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32VpssChnBinding = VPSS_CHN1;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32VpssGrpBinding = (VPSS_GRP)0;

    // VBPool 2 for TDL preprocessing
    stMWConfig.stVBPoolConfig.astVBPoolSetup[2].enFormat = PIXEL_FORMAT_BGR_888_PLANAR;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[2].u32BlkCount = 3;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[2].u32Height = 720;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[2].u32Width = 1280;
    stMWConfig.stVBPoolConfig.astVBPoolSetup[2].bBind = false;

    // Setup VPSS Grp0
    stMWConfig.stVPSSPoolConfig.u32VpssGrpCount = 1;
    stMWConfig.stVPSSPoolConfig.stVpssMode.aenInput[0] = VPSS_INPUT_MEM;
    stMWConfig.stVPSSPoolConfig.stVpssMode.enMode = VPSS_MODE_DUAL;
    stMWConfig.stVPSSPoolConfig.stVpssMode.ViPipe[0] = 0;
    stMWConfig.stVPSSPoolConfig.stVpssMode.aenInput[1] = VPSS_INPUT_ISP;
    stMWConfig.stVPSSPoolConfig.stVpssMode.ViPipe[1] = 0;

    SAMPLE_TDL_VPSS_CONFIG_S *pstVpssConfig = &stMWConfig.stVPSSPoolConfig.astVpssConfig[0];
    pstVpssConfig->bBindVI = true;

    // Assign device 1 to VPSS Grp0, because device1 has 3 outputs in dual mode.
    VPSS_GRP_DEFAULT_HELPER2(&pstVpssConfig->stVpssGrpAttr, stSensorSize.u32Width,
                             stSensorSize.u32Height, VI_PIXEL_FORMAT, 1);
    pstVpssConfig->u32ChnCount = 2;
    pstVpssConfig->u32ChnBindVI = 0;
    VPSS_CHN_DEFAULT_HELPER(&pstVpssConfig->astVpssChnAttr[0], stVencSize.u32Width,
                            stVencSize.u32Height, VI_PIXEL_FORMAT, true);
    // VPSS_CHN_DEFAULT_HELPER(&pstVpssConfig->astVpssChnAttr[0], 368,
    //                         552, VI_PIXEL_FORMAT, true);
    VPSS_CHN_DEFAULT_HELPER(&pstVpssConfig->astVpssChnAttr[1], stVencSize.u32Width,
                            stVencSize.u32Height, VI_PIXEL_FORMAT, true);

    // Get default VENC configurations
    SAMPLE_TDL_Get_Input_Config(&stMWConfig.stVencConfig.stChnInputCfg);
    stMWConfig.stVencConfig.u32FrameWidth = stVencSize.u32Width;
    stMWConfig.stVencConfig.u32FrameHeight = stVencSize.u32Height;

    // Get default RTSP configurations
    SAMPLE_TDL_Get_RTSP_Config(&stMWConfig.stRTSPConfig.stRTSPConfig);

    SAMPLE_TDL_MW_CONTEXT stMWContext = {0};
    s32Ret = SAMPLE_TDL_Init_WM(&stMWConfig, &stMWContext);
    if (s32Ret != CVI_SUCCESS)
    {
        printf("init middleware failed! ret=%x\n", s32Ret);
        return -1;
    }

    cvitdl_handle_t stTDLHandle = NULL;

    // Create TDL handle and assign VPSS Grp1 Device 0 to TDL SDK
    GOTO_IF_FAILED(CVI_TDL_CreateHandle2(&stTDLHandle, 1, 0), s32Ret, create_tdl_fail);

    GOTO_IF_FAILED(CVI_TDL_SetVBPool(stTDLHandle, 0, 2), s32Ret, create_service_fail);

    CVI_TDL_SetVpssTimeout(stTDLHandle, 1000);

    cvitdl_service_handle_t stServiceHandle = NULL;
    GOTO_IF_FAILED(CVI_TDL_Service_CreateHandle(&stServiceHandle, stTDLHandle), s32Ret,
                   create_service_fail);

    GOTO_IF_FAILED(CVI_TDL_OpenModel(stTDLHandle, CVI_TDL_SUPPORTED_MODEL_SCRFDFACE, argv[1]), s32Ret,
                   setup_tdl_fail);

    pthread_t stVencThread, stTDLThread;
    SAMPLE_TDL_VENC_THREAD_ARG_S args = {
        .pstMWContext = &stMWContext,
        .stServiceHandle = stServiceHandle,
    };

    // s32Ret = SAMPLE_PLAT_VO_INIT2();
    // if (s32Ret != CVI_SUCCESS)
    // {
    //     SAMPLE_PRT("vo init failed. s32Ret: 0x%x !\n", s32Ret);
    //     return s32Ret;
    // }

    // CVI_VO_SetChnRotation(0, 0, ROTATION_90);
    // SAMPLE_COMM_VPSS_Bind_VO(0, 0, 0, 0);

    pthread_create(&stVencThread, NULL, run_venc, &args);
    pthread_create(&stTDLThread, NULL, run_tdl_thread, stTDLHandle);

    pthread_join(stVencThread, NULL);
    pthread_join(stTDLThread, NULL);
    printf("%s,%d\r\n",__func__,__LINE__);
setup_tdl_fail:
    CVI_TDL_Service_DestroyHandle(stServiceHandle);
    printf("%s,%d\r\n",__func__,__LINE__);
create_service_fail:
    CVI_TDL_DestroyHandle(stTDLHandle);
    printf("%s,%d\r\n",__func__,__LINE__);
create_tdl_fail:
    SAMPLE_TDL_Destroy_MW(&stMWContext);
    printf("%s,%d\r\n",__func__,__LINE__);

    return 0;
}
