/**
* @file wt61c-dump.c
* @brief 加速度センサ WT61Cの出力をダンプするプログラム
* @author amagai
*
* witmotionのWT61Cが対象。
* ドキュメント類は以下から取得。
* https://github.com/WITMOTION/WT61C-RS232
*
*/

// gcc wt61c-dump.c -lpthread

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <getopt.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <pthread.h>
#include <signal.h>

#define DEFAULT_BAUDRATE B115200

#define WT61CFRAMESIZE 11

// 読み込みステートマシン用の定数
enum { WT61STAT_INIT = 0, WT61STAT_HDR, WT61STAT_BODY };

// 100Hzでデータが出てくるので、
// ファイルの出力で数秒待たされても大丈夫なように
// 10秒分のFIFOを用意する。
#define WT61CFIFOSIZE 1024

/**
 * @brief センサデータの1レコード
 */
typedef struct 
{
    unsigned int seq;           //!< シーケンス番号
    struct timeval tv;          //!< 読み込み時刻
    double ax, ay, az;          //!< 加速度
    double wx, wy, wz;          //!< 角速度
    double r,p,y;               //!< Roll, Pitch, Yaw
    double t;                   //!< Temp
} WT61C_REC;


/**
 * @brief センサ制御用構造体
 */
typedef struct 
{
    int fd;
    int stat;
    int overflow;
    uint8_t linebuf[WT61CFRAMESIZE];
    unsigned int wptr;
    struct timeval ts;

    // 現在読み込み中のレコード
    WT61C_REC rec;

    // FIFO
    WT61C_REC ringbuf[WT61CFIFOSIZE];
    unsigned int rb_rptr;
    unsigned int rb_wptr;
} WT61C_CTRL;


static volatile int AbortAll;

/**
 * @brief  SIGPIPE ハンドラ
 */
static void sigPipeHandler(int sigNum)
{
	AbortAll = 1;
}


/**
 * @brief  SIGINT ハンドラ
 */
static void sigIntHandler(int sigNum)
{
	AbortAll = 1;
}



/**
 * @brief  シリアルポートの初期化
 * @param [in] fd ファイルディスクリプタ
 * @param [in] baud 通信レートの定数
 * @return 成功すれば0
 */
int SerialInit(int fd, int baud)
{
    struct termios tio;

    memset(&tio,0,sizeof(tio));

    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VMIN] = 1;

    cfsetispeed(&tio,baud);
    cfsetospeed(&tio,baud);

    tcsetattr(fd,TCSANOW,&tio);

    return 0;
}


/**
 * @brief WT61C制御構造体の初期化
 * @param [in] wt61c WT61C制御構造体
 * @param [in] fd シリアルポートのファイルディスクリプタ
 * @return 成功すれば0
 */
int WT61C_Init(WT61C_CTRL *wt61c, int fd)
{
    wt61c->fd = fd;
    wt61c->stat = WT61STAT_INIT;
    wt61c->wptr = 0;
    wt61c->rb_rptr = 0;
    wt61c->rb_wptr = 0;
    wt61c->overflow = 0;

    wt61c->rec.ax = 0.0;
    wt61c->rec.ay = 0.0;
    wt61c->rec.az = 0.0;
    wt61c->rec.wx = 0.0;
    wt61c->rec.wy = 0.0;
    wt61c->rec.wz = 0.0;
    wt61c->rec.r = 0.0;
    wt61c->rec.p = 0.0;
    wt61c->rec.y = 0.0;
    wt61c->rec.t = 0.0;
    wt61c->rec.seq = 0;
    wt61c->rec.tv.tv_sec = 0;
    wt61c->rec.tv.tv_usec = 0;

    return 0;
}


/**
 * @brief 現在のセンサデータ値をFIFOに押し込む
 * @param [in] wt61c WT61C制御構造体
 * @return 成功すれば0 オーバーフローが発生すると -1
 */
int WT61C_Push(WT61C_CTRL *wt61c)
{
    int nxt;

    nxt = (wt61c->rb_wptr + 1) % WT61CFIFOSIZE;
    if( nxt == wt61c->rb_rptr )
    {
        wt61c->overflow = 1;
        return -1;
    }

    memcpy(&wt61c->ringbuf[wt61c->rb_wptr], &wt61c->rec, sizeof(wt61c->rec) );
    wt61c->rb_wptr = nxt;

    return 0;
}


/**
 * @brief FIFOからデータを1つ取り出す
 * @param [in] wt61c WT61C制御構造体
 * @param [out] rec 取り出した値の格納先
 * @return 成功すれば0。FIFOが空なら-1
 */
int WT61C_Pop(WT61C_CTRL *wt61c, WT61C_REC *rec)
{
    if( wt61c->rb_rptr == wt61c->rb_wptr )
    {
        return -1;
    }

    memcpy(rec, &wt61c->ringbuf[wt61c->rb_rptr], sizeof(WT61C_REC) );
    wt61c->rb_rptr = (wt61c->rb_rptr + 1) % WT61CFIFOSIZE;
    return 0;
}


/**
 * @brief 2バイトを符号付きshortに変換
 * @param [in] vl 下位8bit
 * @param [in] vh 上位8bit
 * @return 変換値
 */
static inline int16_t WT61C_B2S(uint8_t vl, uint8_t vh)
{
    int16_t s;

    s = ((int16_t)vh << 8 | vl);

    return s;
}


/**
 * @brief 現在時刻表示 (デバッグ用)
 */
void ShowTS(void)
{
    struct timeval tv;
    gettimeofday( &tv, NULL);
    printf("%ld.%06ld: ", tv.tv_sec, tv.tv_usec);
}


/**
 * @brief 加速度データの読み込み
 * @param [in] wt61c WT61C制御構造体
 */
int WT61C_ParseA(WT61C_CTRL *wt61c)
{
    int16_t v;

    v = WT61C_B2S(wt61c->linebuf[2], wt61c->linebuf[3]);
    wt61c->rec.ax = (double)v / 32768.0 * 16.0 * 9.8;
    v = WT61C_B2S(wt61c->linebuf[4], wt61c->linebuf[5]);
    wt61c->rec.ay = (double)v / 32768.0 * 16.0 * 9.8;
    v = WT61C_B2S(wt61c->linebuf[6], wt61c->linebuf[7]);
    wt61c->rec.az = (double)v / 32768.0 * 16.0 * 9.8;
    v = WT61C_B2S(wt61c->linebuf[8], wt61c->linebuf[9]);
    wt61c->rec.t = (double)v / 340.0 + 36.53;

//    ShowTS();
//    printf("A: %f, %f, %f\n", wt61c->ax, wt61c->ay, wt61c->az);
    return 0;
}


/**
 * @brief 角速度データの読み込み
 * @param [in] wt61c WT61C制御構造体
 */
int WT61C_ParseW(WT61C_CTRL *wt61c)
{
    int16_t v;

    v = WT61C_B2S(wt61c->linebuf[2], wt61c->linebuf[3]);
    wt61c->rec.wx = (double)v / 32768.0 * 2000.0;
    v = WT61C_B2S(wt61c->linebuf[4], wt61c->linebuf[5]);
    wt61c->rec.wy = (double)v / 32768.0 * 2000.0;
    v = WT61C_B2S(wt61c->linebuf[6], wt61c->linebuf[7]);
    wt61c->rec.wz = (double)v / 32768.0 * 2000.0;
    v = WT61C_B2S(wt61c->linebuf[8], wt61c->linebuf[9]);
    wt61c->rec.t = (double)v / 340.0 + 36.53;
//    ShowTS();
//    printf("W: %f, %f, %f\n", wt61c->wx, wt61c->wy, wt61c->wz);

    return 0;
}


/**
 * @brief RPYデータの読み込み
 * @param [in] wt61c WT61C制御構造体
 */
int WT61C_ParseR(WT61C_CTRL *wt61c)
{
    int16_t v;

    v = WT61C_B2S(wt61c->linebuf[2], wt61c->linebuf[3]);
    wt61c->rec.r = (double)v / 32768.0 * 180.0;
    v = WT61C_B2S(wt61c->linebuf[4], wt61c->linebuf[5]);
    wt61c->rec.p = (double)v / 32768.0 * 180.0;
    v = WT61C_B2S(wt61c->linebuf[6], wt61c->linebuf[7]);
    wt61c->rec.y = (double)v / 32768.0 * 180.0;
    v = WT61C_B2S(wt61c->linebuf[8], wt61c->linebuf[9]);
    wt61c->rec.t = (double)v / 340.0 + 36.53;

//    ShowTS();
//    printf("R: %f, %f, %f\n", wt61c->r, wt61c->p, wt61c->y);

    return 0;
}


/**
 * @brief 1バイト処理
 * @param [in] wt61c
 * @return R,P,Yデータの更新があった場合は1、それ以外は0
*/
int WT61C_ParseByte(WT61C_CTRL *wt61c, uint8_t dat)
{
    int updated;
    struct timeval tv;
    uint8_t sum;
    int i;

//    printf("%02x ", dat);
    // err chk
    if( wt61c->wptr >= WT61CFRAMESIZE )
    {
        // err
        wt61c->stat = WT61STAT_INIT;
    }

    updated = 0;
    switch(wt61c->stat)
    {
        case WT61STAT_INIT:
            wt61c->wptr = 0;
            if( dat == 0x55 )
            {
                gettimeofday( &tv, NULL);
                wt61c->ts.tv_sec = tv.tv_sec;
                wt61c->ts.tv_usec = tv.tv_usec;
                wt61c->linebuf[wt61c->wptr++] = dat;
                wt61c->stat = WT61STAT_HDR;
            }
            break;
        
        case WT61STAT_HDR:
            if( dat == 0x51 || dat == 0x52 || dat == 0x53)
            {
                wt61c->linebuf[wt61c->wptr++] = dat;
                wt61c->stat = WT61STAT_BODY;
            }
            else
            {
                wt61c->stat = WT61STAT_INIT;
            }
            break;

        case WT61STAT_BODY:
            wt61c->linebuf[ wt61c->wptr++ ] = dat;
            if(wt61c->wptr == WT61CFRAMESIZE)
            {
                // 
                sum = 0;
                for( i = 0; i < 10; i++)
                {
                    sum += wt61c->linebuf[i];
                }

                if( sum != wt61c->linebuf[10])
                {
                    fprintf(stderr, "Checksum error");
                }
                else
                {
                    // ready to parse
                    switch( wt61c->linebuf[1])
                    {
                        case 0x51:
                            WT61C_ParseA(wt61c);
                            wt61c->rec.tv.tv_sec = wt61c->ts.tv_sec;
                            wt61c->rec.tv.tv_usec = wt61c->ts.tv_usec;
                            break;

                        case 0x52:
                            WT61C_ParseW(wt61c);
                            break;

                        case 0x53:
                            WT61C_ParseR(wt61c);
                            // 加速度データから順に記録されていない場合は
                            // 無効データとする
                            if( wt61c->rec.tv.tv_sec != 0 )
                            {
                                updated = 1;
                            }
                            break;

                        default:
                            break;
                    }
                }

                wt61c->stat = WT61STAT_INIT;
            }
            
            break;
        
        default:
            wt61c->stat = WT61STAT_INIT;
            wt61c->wptr = 0;
            break;
    }

    return updated;
}


/**
 * @brief センサデータをfoutに出力 (デバッグ用)
 * @param [in] rec WT61Cデータ
 */
void WT61C_Dump(WT61C_REC *rec, FILE *fout)
{
    fprintf(fout, "Seq: %u\n", rec->seq);
    fprintf(fout, "TS: %ld.%06ld\n", rec->tv.tv_sec, rec->tv.tv_usec);
    fprintf(fout, "A:%f, %f, %f\n", rec->ax, rec->ay, rec->az);
    fprintf(fout, "W:%f, %f, %f\n", rec->wx, rec->wy, rec->wz);
    fprintf(fout, "R:%f, %f, %f\n", rec->r, rec->p, rec->y);
    fprintf(fout, "T:%f\n", rec->t);
}


/**
 * @brief センサデータをCSV形式でfoutに出力
 * @param [in] rec WT61Cデータ
 * @param [in] fout 出力先ファイルポインタ
 * 
 * データは基本的に小数点以下3桁の精度しか無い。温度は2桁。
 */
void WT61C_DumpCSV(WT61C_REC *rec, FILE *fout)
{
    fprintf(fout, "%u,", rec->seq);
    fprintf(fout, "%ld.%06ld,", rec->tv.tv_sec, rec->tv.tv_usec);
    fprintf(fout, "%.3f,%.3f,%.3f,", rec->ax, rec->ay, rec->az);
    fprintf(fout, "%.3f,%.3f,%.3f,", rec->wx, rec->wy, rec->wz);
    fprintf(fout, "%.3f,%.3f,%.3f,", rec->r, rec->p, rec->y);
    fprintf(fout, "%.2f\n", rec->t);
}


/**
 * @brief CSVのヘッダ行の出力
 * @param [in] fout 出力先ファイルポインタ
 */
void WT61C_DumpHeader(FILE *fout)
{
    fprintf(fout, "seq,");
    fprintf(fout, "time,");
    fprintf(fout, "ax,ay,az,");
    fprintf(fout, "wx,wy,wz,");
    fprintf(fout, "roll,pitch,yaw,");
    fprintf(fout, "temp\n");
}


/**
 * @brief シリアルポートから読み込んだバイト列を処理
 * @param [in] wt61c WT61C制御構造体
 * @param [in] buf 読み込みバッファ
 * @param [in] datlen バッファの中身のバイト数
 */
int WT61C_ParseData(WT61C_CTRL *wt61c, uint8_t buf[], int datlen)
{
    int i;

    for(i = 0; i < datlen; i++)
    {
        if( WT61C_ParseByte(wt61c, buf[i]) == 1 )
        {
            WT61C_Push(wt61c);
            wt61c->rec.seq++;
        }
    }

    return 0;
}


/**
 * @brief デバイスのオープンと初期化
 * @param [in] wt61c WT61C制御構造体
 * @param [in] ttyname シリアルポートのデバイス名
 * @return 成功すれば0
 */
int WT61C_Open(WT61C_CTRL *wt61c, char *ttyname)
{
    int fd;
    uint8_t rxbuf[64];
    struct pollfd pfd;
    int rn;

    fd = open(ttyname, O_RDWR);
    if( fd < 0 )
    {
        return -1;
    }

    SerialInit(fd, DEFAULT_BAUDRATE);
    WT61C_Init(wt61c, fd);

    // ゴミを読み捨てる
    pfd.fd = wt61c->fd;
    pfd.events = POLLIN;
    while( poll( &pfd, 1, 200) > 0 )
    {
        rn = read(wt61c->fd, rxbuf, sizeof(rxbuf));
        if( rn <= 1 )       // 取り出せたデータが1バイト以下なら空になったという扱い
        {
            break;
        }
    }

    return 0;
}


/**
 * @brief デバイスのクローズ
 * @param [in] wt61c WT61C制御構造体
 * @return 成功すれば0
 */
int WT61C_Close(WT61C_CTRL *wt61c)
{
    close(wt61c->fd);
    wt61c->fd = 0;
    return 0;
}


/**
 * @brief デバイスをポーリングしてデータを得たらFIFOへ詰め込んでいく
 * @param [in] wt61c WT61C制御構造体
 * @return 成功すれば0。タイムアウトやオーバーフローが発生したら0以外。
 * 
 * データを1秒読み込めなかったらタイムアウト。
 * シグナル等で中断されるまでこの処理は抜けない。
 */
int WT61C_Poll(WT61C_CTRL *wt61c)
{
    struct pollfd pfd;
    uint8_t rxbuf[256];
    int rn;

    pfd.fd = wt61c->fd;
    pfd.events = POLLIN;

    while( poll( &pfd, 1, 1000) > 0 )
    {
        if( AbortAll )
            return 0;
        rn = read(wt61c->fd, rxbuf, sizeof(rxbuf));
//        printf("%d\n", rn);
        if( rn > 0 )
        {
            WT61C_ParseData(wt61c, rxbuf, rn);
        }

        if( wt61c->overflow != 0 )
        {
            return 1;
        }
    }

    // Timeout
    AbortAll = 1;
    fprintf(stderr, "Timeout\n");
    return 1;
}


/**
 * @brief センサからデータを読み込んでFIFOに投入するスレッド
 * @param [in] p WT61C制御構造体
*/
static void *WT61C_ThrSrc(void *p)
{
    static WT61C_CTRL *wt61c;

    wt61c = (WT61C_CTRL *)p;

    WT61C_Poll(wt61c);

    return 0;
}


/**
 * @brief センサからデータを読み込んでfoutへ出力
 * @param [in] ttyname デバイス名。ex. /dev/ttyUSB0
 * @param [in] fout 出力先のファイルポインタ
 * @param [in] ntimes 何回出力したら停止するか。0で無制限。
*/
int DumpSensor(char *ttyname, FILE *fout, int ntimes)
{
    static WT61C_CTRL wt61c;
    WT61C_REC rec;
	struct sigaction sigPipeHandl;
	struct sigaction sigIntHandl;
    pthread_t thr_src;
    unsigned int cnt;

    AbortAll = 0;

    /* シグナルの種類をセット（失敗すると -1 を返す） */
	sigPipeHandl.sa_handler = sigPipeHandler;
	sigPipeHandl.sa_flags   = 0;
	if (sigaction(SIGPIPE, &sigPipeHandl, 0) < 0)
	{
		fprintf(stderr, "sigaction() failed\n");
        return 1;
	}
	
	/* シグナルの種類をセット（失敗すると -1 を返す） */
	sigIntHandl.sa_handler = sigIntHandler;
	sigIntHandl.sa_flags   = 0;
	if (sigaction(SIGINT, &sigIntHandl, 0) < 0)
	{
		fprintf(stderr, "sigaction() failed\n");
        return 1;
	}

    if( WT61C_Open(&wt61c, ttyname) != 0 )
    {
        fprintf(stderr, "Failed to open %s\n", ttyname);
        return 1;
    }

    pthread_create( &thr_src, NULL, WT61C_ThrSrc, &wt61c );

    cnt = 0;
    while( AbortAll == 0 )
    {
        while( AbortAll == 0 && WT61C_Pop(&wt61c, &rec) == 0 )
        {
            WT61C_DumpCSV(&rec, fout);
            if( ntimes > 0 )
            {
                if( ++cnt >= ntimes )
                {
                    AbortAll = 1;
                    break;
                }
            }
        }
        usleep(10000);
    }
    pthread_join( thr_src, NULL );
    WT61C_Close(&wt61c);

//	fprintf(stderr, "Terminated\n");
    return 0;
}


/**
 * @brief 使い方の表示
*/
void ShowUsage(void)
{
    fprintf(stderr, "Usage: wt61c-dump <-i DevName> [-o OutFile] [-n NTimes] [-H]\n");
    fprintf(stderr, "ex: wt61c-dump -i /dev/ttyUSB0 -o record.csv\n");
    fprintf(stderr, "Options\n");
    fprintf(stderr, "-i DevName : Device name of serial port\n");
    fprintf(stderr, "-o OutFile : Output file name. Without this option, stdout will be used.\n");
    fprintf(stderr, "-n NTimes  : Terminate after reading NTimes times. Setting this option 0 or omitting this option causes infinite reading.\n");
    fprintf(stderr, "-H         : Add CSV header line.\n");
    fprintf(stderr, "-h         : Show this help and quit.\n");
}


int main(int argc, char **argv)
{
    int c;
    char ttyname[64];
    FILE *fout;
    int ntimes;
    int hdr;

    fout = stdout;
    ntimes = 0;
    ttyname[0] = 0;
    hdr = 0;

    while ((c = getopt (argc, argv, "Hhi:o:n:")) != -1)
	{
		switch( c )
		{
            case 'h':
                ShowUsage();
                return 0;

            case 'H':
                hdr = 1;
                break;

            case 'n':
                ntimes = atoi(optarg);
                if( ntimes < 0 )
                {
                    fprintf(stderr, "Invalid -n option\n");
                    return 1;
                }
                break;

            case 'i':
                strncpy(ttyname, optarg, sizeof(ttyname));
                ttyname[sizeof(ttyname) -1] = 0;
                break;
                
            case 'o':
                fout = fopen(optarg, "w");
                if( fout == NULL )
                {
                    fprintf(stderr, "Failed to create %s\n", optarg);
                    return 1;
                }
                break;

            case '?':
                if (isprint (optopt) )
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                
                ShowUsage();
                return 1;
		}
	}

    if( ttyname[0] == 0 )
    {
        ShowUsage();
        return 1;
    }

    if( hdr )
    {
        WT61C_DumpHeader(fout);
    }

    return DumpSensor(ttyname, fout, ntimes);
}
