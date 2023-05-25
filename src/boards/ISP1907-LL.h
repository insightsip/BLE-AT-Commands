 /******************************************************************************
 * @file    ISP1907-LL.h
 * @author  Insight SiP
 * @brief   ISP1907-LL board specific file.
 *
 * @attention
 *  THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef ISP1907_LL_H
#define ISP1907_LL_H

// Boards specific pinout
#define PIN_UART_RX             8       // Pin 32
#define PIN_UART_TX             17      // Pin 34
#define PIN_UART_CTS            3       // Pin 38
#define PIN_UART_RTS            5       // Pin 36
#define PIN_SER_PKT_FW_SELECT   11      // Pin 48

// Capabilities & specificities
#define BLE_MAX_TXP_8DBM        0
#define BLE_CAP_PHY_CODED       0

// Others
#define MODULE_NAME             "ISP1907-LL"


#endif // ISP1907_LL_H
