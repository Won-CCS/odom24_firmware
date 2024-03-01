#include <cmath>
#include <bit>
#include <numbers>
#include <main.h>
#include <gpio.h>
#include <tim.h>
#include <can.h>
#include <spi.h>
#include<cstring>

//割り込みフラグ
uint8_t interrupt = 0;
//割り込み回数カウント
uint16_t interruptCount = 0;

//プロトタイプ宣言
void can_send(uint16_t, int16_t, int16_t, int16_t, uint16_t);
namespace MPU9250{
	struct vector3;
	class mpu9250;
}
struct encoder;
encoder get_encoder_counts();

namespace MPU9250{
	//3軸の情報を格納する型
	struct vector3{
		int16_t x = 0;
		int16_t y = 0;
		int16_t z = 0;
	};
    class mpu9250{
        public:
        //MPU9250の読み込み関数
        bool read(uint8_t reg, uint8_t *Rdata/*レジスタの番号を書くとその情報がRdataに帰ってくる*/){
            uint8_t rx_data[2] = {0,0};
            uint8_t tx_data[2];
            //上位8bitでレジスタを指定して下位8bitでデータを指定
            //MPU9250のレジスタの値を読むときはレジスタの上位1bitを１にする
            tx_data[0] = reg | 0x80;
            tx_data[1] = 0x00;

            //CSをLowにして通信中状態へ切り替え
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
            //通信が成立したら受け取ったデータをRdataに入れ、CSをHighにしてアイドル状態に切り替えTrueを返す
            if (HAL_SPI_TransmitReceive(&hspi1,tx_data,rx_data,2,1) == HAL_OK){
                *Rdata = rx_data[1];
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
                return true;
            //通信が失敗したらCSをHighにしてアイドル状態に切り替えFalseを返す
            }else{
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
                return false;
            }
        }

        //MPU9250の書き込み関数
        bool write(uint8_t reg, uint8_t Tdata/*レジスタの番号を書いてTdataのデータを送る*/){
                uint8_t rx_data[2];
                uint8_t tx_data[2];

                //上位8bitでレジスタを指定して下位8bitでデータを指定
                tx_data[0] = reg;
                tx_data[1] = Tdata;

                //CSをLowにして通信中状態へ切り替え
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
                //通信が成立したらデータを送信し、CSをHighにしてアイドル状態に切り替えTrueを返す
                if (HAL_SPI_TransmitReceive(&hspi1,tx_data,rx_data,2,1) == HAL_OK){
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
                    return true;
                //通信が失敗したらCSをHighにしてアイドル状態に切り替えFalseを返す
                }else{
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
                    return false;
                }
            }

        //MPUのレジスタの設定をする関数(最初に一度だけ呼び出せばいい)
        void init(void)
        {
            //見やすさの為にレジスタ番号を格納したわけだけどいらなかったかも
            uint8_t reg108 = 0x6C;
            uint8_t reg107 = 0x6B;
            uint8_t reg106 = 0x6A;
            uint8_t reg56 = 0x38;
            uint8_t reg29 = 0x1D;
            uint8_t reg28 = 0x1C;
            uint8_t reg27 = 0x1B;
            uint8_t reg26 = 0x1A;

            //最大100ms待たないと書き込みも読み込みもできないので一応120ms待つ
            HAL_Delay(120);

            //Power Management 2
			//ジャイロセンサ、加速度センサ全軸有効化
			write(reg108,0x00);
			HAL_Delay(1);

            //Power Management 1
            //内部レジスタをリセットしてデフォルトに設定(reg107とreg117以外すべて0x00に)
            write(reg107, 0x80);
            HAL_Delay(1);

            //User Control
            //I2Cマスター有効化
            //I2CのSlaveモジュールを使えなくし、シリアルインターフェースをSPIモードにする
            //ジャイロ・デジタル信号経路、加速度デジタル信号経路、温度デジタル信号経路をすべてリセットする。全てのセンサーレジスタもクリアする。
            write(reg106, 0x31);
            HAL_Delay(1);

            //Interrupt Enable
			write(reg56, 0x01);
			HAL_Delay(1);

            //Accelerometer Configuration2
			//加速度センサのローパスフィルタの設定
			//加速度センサの周波数帯は99Hzでデータ出力は1kHz
			write(reg29, 0x02);
			HAL_Delay(1);

			//Accelerometer Configuration
			//加速度センサを±16gに設定
			write(reg28, 0x18);
			HAL_Delay(1);

            //Gyroscope Configuration
            //ジャイロを+2000dpsに設定、サンプリング周波数は8kHz
            write(reg27, 0x18);
            HAL_Delay(1);

            //Configuration
            write(reg26, 0x00);
            HAL_Delay(1);
        }

        //上位8bitと下位8bitを合体させる関数
        int16_t data_combine(uint8_t high_data, uint8_t low_data){
            /*
            * (high_data<<8|low_data)の操作で0b1000000000000000のようなビットが
            * 入っていた時これはシフト演算子の仕様でint型になり32768となる、
            * これをint16_tに入れようとすると最大値が32767なのでオーバーフロー
            * ビット的には入れられそうでも整数で見たときこの操作はできないとわかる。
            * なので一旦uint16_tに入れてビット幅をそろえてからbit_castでint16_tに入れる
            */
            uint16_t merged = (high_data<<8|low_data);
            int16_t data = std::bit_cast<int16_t>(merged);
            return data;
        }

        //ジャイロオフセット計算関数(最初に一回だけ計算)
        vector3 gyro_calib()
        {
            vector3 gyroOffset = {0, 0, 0};
            vector3 gyro;
            int32_t sumX = 0;
            int32_t sumY = 0;
            int32_t sumZ = 0;
            for(int i = 0; i < 1000; i++){
                gyro = gyro_read(gyroOffset);
                sumX += gyro.x;
                sumY += gyro.y;
                sumZ += gyro.z;
                HAL_Delay(1);
            }
            gyroOffset.x = sumX / 1000;
            gyroOffset.y = sumY / 1000;
            gyroOffset.z = sumZ / 1000;
            return gyroOffset;
        }

        //ジャイロ読み出し関数
        vector3 gyro_read(vector3 gyroOffest)
        {
            vector3 gyro;
            uint8_t high_data = 0x00;
            uint8_t low_data = 0x00;

            read(static_cast<uint8_t>(67), &high_data);
            read(static_cast<uint8_t>(68), &low_data);
            gyro.x = data_combine(high_data, low_data) - gyroOffest.x;

            read(static_cast<uint8_t>(69), &high_data);
            read(static_cast<uint8_t>(70), &low_data);
            gyro.y = data_combine(high_data, low_data) - gyroOffest.y;

            read(static_cast<uint8_t>(71), &high_data);
            read(static_cast<uint8_t>(72), &low_data);
            gyro.z = data_combine(high_data, low_data) - gyroOffest.z;

            return gyro;
        }

        //加速度読み出し関数
        vector3 acc_read(vector3 accOffest)
        {
            vector3 acc;
            uint8_t high_data = 0x00;
            uint8_t low_data = 0x00;

            read(static_cast<uint8_t>(59), &high_data);
            read(static_cast<uint8_t>(60), &low_data);
            acc.x = data_combine(high_data, low_data);

            read(static_cast<uint8_t>(61), &high_data);
            read(static_cast<uint8_t>(62), &low_data);
            acc.y = data_combine(high_data, low_data);

            read(static_cast<uint8_t>(63), &high_data);
            read(static_cast<uint8_t>(64), &low_data);
            acc.z = data_combine(high_data, low_data);

            return acc;
        };
    };
}

struct encoder{
    uint16_t encoder1;
    uint16_t encoder2;
};

encoder get_encoder_counts(){
    encoder rawCount;
	//タイマーの値をレジスタから読んでくる
	rawCount.encoder1 = -(TIM2->CNT);
	rawCount.encoder2 = -(TIM3->CNT);
	//エンコーダが使っているタイマのカウント値を0に戻しておく
	TIM2->CNT = 0;
	TIM3->CNT = 0;
	return rawCount;
}

void can_send(uint16_t id, int16_t x, int16_t y, int16_t z, uint16_t count){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8];
	//CAN送信
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		TxHeader.StdId = id;                 // CAN ID
		TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
		TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
		TxHeader.DLC = 8;                       // データ長は8バイトに
		TxHeader.TransmitGlobalTime = DISABLE;  // フレーム送信の開始時にタイムスタンプカウンター値をキャプチャしない
		std::memcpy(TxData, &x, sizeof(x));
		std::memcpy(&TxData[2], &y, sizeof(y));
		std::memcpy(&TxData[4], &z, sizeof(z));
		std::memcpy(&TxData[6], &count, sizeof(count));
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	}else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	}
}

//タイマの割り込み時の処理
//周期は 2/(72MHz/36000) = 2/2000 = 1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim1){
		//割り込みフラグを立てる
		interrupt = 1;
		//割り込みカウントを1加算
		interruptCount += 1;
	}
}

extern"C" void main_cpp()
{
	//準備中のインジケータを発光(オレンジLED)
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);

    MPU9250::vector3 accOffset;
    MPU9250::vector3 gyroOffset;
    MPU9250::vector3 gyro;
	MPU9250::vector3 acc;

	encoder encoderRawCount;
    //MPU9250クラスのインスタンスを作成
    MPU9250::mpu9250 mpu9250;
    //mpu9250のレジスタ設定
    mpu9250.init();
    //ジャイロセンサのオフセットを計算
    gyroOffset = mpu9250.gyro_calib();
    //エンコーダのカウントをスタート
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	//CAN通信設定
	HAL_CAN_Start(&hcan);
    //割り込みタイマ設定(割り込み周期はiocファイルで設定、今0.125秒周期)
	HAL_TIM_Base_Start_IT(&htim1);

	//準備中のインジケータを消灯(オレンジLED)
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);

	//処理開始
	while(1)
	{
		//タイマ割り込みのフラグによって周期を制御
		if(interrupt == 1){
			//エンコーダの値を取得
			encoderRawCount = get_encoder_counts();
			//センサ値取得
			gyro = mpu9250.gyro_read(gyroOffset);
			acc = mpu9250.acc_read(accOffset);
			//タイマ割り込みフラグを降ろす
			interrupt = 0;
		}
		//CANで送信
		can_send(0x555, gyro.x, gyro.y, gyro.z, encoderRawCount.encoder1);
		can_send(0x556, acc.x, acc.y, acc.z, encoderRawCount.encoder2);
		//ループが回っているか+割り込みが動いているかのインジケータ(緑LED)を点滅させる
		if(interruptCount >= 1000){
			interruptCount = 0;
		}else if(interruptCount >= 200){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		}else if(interruptCount >=0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		}
    }
}