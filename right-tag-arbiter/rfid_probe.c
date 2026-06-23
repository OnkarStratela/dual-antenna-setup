// rfid_probe.c
// ---------------------------------------------------------------------------
// Hardware sanity probe. It does NOT arbitrate anything -- it answers one
// question: are Source_0 and Source_1 actually two DIFFERENT physical
// antennas pointing at two different places, or are they the same RF?
//
// Symptom that prompted this: at 316 mW both sources read the same tag at
// nearly the same RSSI, and one source is always ~3 dB stronger regardless of
// where the mug sits. That is either (1) both logical sources mapping to the
// same antenna port, or (2) a fixed cable/gain bias between two ports that are
// not giving spatial separation. This tool distinguishes the two.
//
// It prints:
//   * the reader model/serial/firmware,
//   * which read-points (Ant0..Ant3) belong to each Source_0..Source_3,
//   * a quick per-source inventory (read count + min/avg/max RSSI) so you can
//     watch how each source's RSSI changes as you move the mug.
//
// Usage:
//   ./rfid_probe            probe at default power
//   ./rfid_probe <mW>       probe at <mW> (1..316)
// ---------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>

#include "CAENRFIDLib_Light.h"
#include "host.h"

#define GC_PORT          "/dev/ttyACM0"
#define GC_BAUDRATE      921600
#define DEFAULT_POWER_MW 100
#define MIN_POWER_MW     1
#define MAX_POWER_MW     316
#define ROUNDS           20      // inventories per source per refresh
#define CYAN  "\033[0;36m"
#define YEL   "\033[0;33m"
#define GRN   "\033[0;32m"
#define RED   "\033[0;31m"
#define RST   "\033[0m"

static volatile int running = 1;
static void on_sigint(int s){ (void)s; running = 0; }

static bool parse_power(const char *s, uint32_t *out){
    if(!s||!*s) return false; char *e=NULL; long v=strtol(s,&e,10);
    if(e==s||*e) return false; if(v<MIN_POWER_MW||v>MAX_POWER_MW) return false;
    *out=(uint32_t)v; return true;
}

int main(int argc, char **argv){
    uint32_t power = DEFAULT_POWER_MW;
    if(argc==2 && !parse_power(argv[1], &power)){
        fprintf(stderr,"Usage: %s [mW 1..316]\n", argv[0]); return 1;
    }

    CAENRFIDReader reader = {
        .connect=_connect, .disconnect=_disconnect, .tx=_tx, .rx=_rx,
        .clear_rx_data=_clear_rx_data, .enable_irqs=_enable_irqs,
        .disable_irqs=_disable_irqs
    };
    RS232_params pp = { .com=GC_PORT, .baudrate=GC_BAUDRATE, .dataBits=8,
                        .stopBits=1, .parity=0, .flowControl=0 };

    signal(SIGINT, on_sigint);

    printf(CYAN "===== RFID Hardware Probe =====" RST "\n");
    printf("Port  : %s @ %d baud\nPower : %u mW\n\n", GC_PORT, GC_BAUDRATE, power);

    if(CAENRFID_Connect(&reader, CAENRFID_RS232, &pp) != CAENRFID_StatusOK){
        printf(RED "ERROR: could not connect. Try: sudo chmod 666 %s" RST "\n", GC_PORT);
        return -1;
    }
    char model[64]={0}, serial[64]={0}, fw[MAX_FWREL_LENGTH+1]={0};
    if(CAENRFID_GetReaderInfo(&reader, model, serial)==CAENRFID_StatusOK)
        printf("Reader: %s  Serial: %s\n", model, serial);
    if(CAENRFID_GetFirmwareRelease(&reader, fw)==CAENRFID_StatusOK)
        printf("Firmware: %s\n", fw);
    CAENRFID_SetPower(&reader, power);

    const char *srcs[4] = {"Source_0","Source_1","Source_2","Source_3"};
    const char *ants[4] = {"Ant0","Ant1","Ant2","Ant3"};

    // ---- 1) Source -> read-point (physical antenna) mapping --------------
    printf("\n" YEL "Source -> antenna(read-point) mapping:" RST "\n");
    for(int s=0; s<4; s++){
        printf("  %-9s : ", srcs[s]);
        bool any=false;
        for(int a=0; a<4; a++){
            uint16_t present=0;
            CAENRFIDErrorCodes ec = CAENRFID_isReadPointPresent(
                &reader, (char*)ants[a], (char*)srcs[s], &present);
            if(ec==CAENRFID_StatusOK && present){ printf("%s ", ants[a]); any=true; }
        }
        if(!any) printf("(none)");
        printf("\n");
    }
    printf(YEL "  ^ If Source_0 and Source_1 list the SAME antenna (or both list\n"
           "    the same port), they are NOT two separate antennas -- that is the\n"
           "    root cause and no software can separate them.\n" RST);

    // ---- 2) Live per-source read count + RSSI ----------------------------
    printf("\n" YEL "Live per-source reads (move the mug between antennas; Ctrl+C to stop):" RST "\n");
    printf("Each refresh runs %d inventories per source.\n\n", ROUNDS);

    while(running){
        for(int s=0; s<2 && running; s++){        // only the two used sources
            int    reads=0;
            int16_t rmin=0, rmax=0; long rsum=0;
            char    epc_last[2*MAX_ID_LENGTH+1] = "--";

            for(int r=0; r<ROUNDS && running; r++){
                CAENRFIDTagList *list=NULL, *node; uint16_t n=0;
                CAENRFIDErrorCodes ec = CAENRFID_InventoryTag(
                    &reader, (char*)srcs[s], 0,0,0, NULL,0, RSSI|PHASE, &list, &n);
                node=list;
                while(node){
                    if(ec==CAENRFID_StatusOK){
                        int16_t v=node->Tag.RSSI;
                        if(reads==0){ rmin=rmax=v; } else { if(v<rmin)rmin=v; if(v>rmax)rmax=v; }
                        rsum+=v; reads++;
                        // remember a short EPC tail
                        int L=node->Tag.Length; int st=(L>3)?L-3:0;
                        char *p=epc_last; for(int i=st;i<L;i++){ sprintf(p,"%02X",node->Tag.ID[i]); p+=2;} *p=0;
                    }
                    CAENRFIDTagList *nx=node->Next; free(node); node=nx;
                }
            }

            const char *col = (s==0)?GRN:RED;
            if(reads>0)
                printf("%s%-9s" RST " reads=%2d/%d  rssi[min/avg/max]= %.1f / %.1f / %.1f dBm  ...%s\n",
                       col, srcs[s], reads, ROUNDS,
                       rmin/10.0, (rsum/(double)reads)/10.0, rmax/10.0, epc_last);
            else
                printf("%s%-9s" RST " reads= 0/%d  (no tag seen)\n", col, srcs[s], ROUNDS);
        }
        printf("----\n");
        usleep(150*1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("\n[probe] Disconnected.\n");
    return 0;
}
