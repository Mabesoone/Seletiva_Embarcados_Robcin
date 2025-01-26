//CODIGO DA ENTRAGA FINAL DE EDUARDO MABESOONE MELO EMM4
// 

#include <iostream>
#include <MPU6050/MPU6050.h>
#include "mbed.h"
#include <fstream>
#include <cmath>
#include <vector>
#include <sstream>
#define TOLERANCE 1.5

#define MPU_SDA PB_9
#define MPU_SCL PB_8
//Esses aqui foram os pinos quando testei sem ser no robo e sim em outro MPU6050
//No robo mesmo seria MPU_SDA PF_0 e MPU_SCL PF_1

static BufferedSerial serial_port(USBTX, USBRX, 230400);
FileHandle* mbed::mbed_override_console(int fd) {
  return &serial_port;
}
class Giroscopio 
{
    public:
        Giroscopio(); 
        void init();
        void Process_gyro(); //Método para processar os dados do giroscopio e converter essa leitura para o formato correto (RAD/S , RAD) 
    private:
        MPU6050 MPU_; 
        void Calibration(); //Método para calibrar o giroscopio , para identificar  se a leitura está enviesada(offsets) para que as leituras futuras não estejam enviesadas
        void Update_flag(); //Método para dar update na flag do giroscopio , a flag determina se o giroscopio deve ser lido ou não
        void Stop();
        bool stop_flag = false;//Flag pra fazer o programa parar de rodar , na pratica nao se usaria ela , mas só pra não ficar rodando o codigo por muito tempo 
        bool gyro_flag = false; //Flag para ver se o giroscopio precisa ser analisado
        bool sensor_flag = false; //Flag para ver se a IMU foi inicializada corretamente 
        int numberSamples = 5; //Quantidade de samples usadas por vez para garatir o minimo de ruido possivel sem comprometer o tempo de resposta
        int flag_timer = 5; // Quanto tempo pra dar update na flag para puder processar o imu
        double calibration_result = 0; //Resultado da Calibração
        double Gyro_angle = 0;
        double Gyro_angle_cumulativo = 0; 
        double Gyro_speed = 0; // Resultados finais 
        Ticker Flag_update; //Ticker para chamar periodicamente o metodo de dar update na flag do gyro
        Ticker Gyro_process; //Ticker para chamar periodicamente o metodo de processar o giroscopio
        Ticker Stop_reading; //Pra não ficar lendo infinitamente do sensor , já que o projeto pega isoladamente a parte do giroscopio e não em uma partida , isso aqui é só pra ter controle de quando o codigo vai parar de rodar
        Timer timer; //Timer para contar quanto tempo foi passado lendo os samples para poder calcular o deslocamento angular
};

Giroscopio::Giroscopio():MPU_(MPU_SDA, MPU_SCL)
{

} //pegando os fios do i2c

void Giroscopio::init()
{
    sensor_flag = MPU_.initialize(); //inicilaiza o sensor
    if(sensor_flag) //checando se o IMU foi inicializado corretamente
    {
        Calibration();
        //primeiro é chamado a função para calibrar o bias do sensor enquanto o robo está parado , e poder evitar que esse offset enviese leitura futuras quando o robo estiver se movendo
        Flag_update.attach(callback(this, &Giroscopio::Update_flag), chrono::milliseconds(flag_timer));
        Stop_reading.attach(callback(this, &Giroscopio::Stop), chrono::milliseconds(60000));//60 segundos joga o stop flag pra true
        //Depois que terminar a calibração os metodos de update de flag e de processar o gyro são agendados para rodar em intervalos fixos de tempo
    }
    else 
    {
         std::cout << "ERRO, falha ao inicializar o sensor";
         exit(1);
         //se aconteceu um erro ao inicializar o sensor o código é abortado
    }
}

void Giroscopio::Stop()
{
    stop_flag = true;
}

void Giroscopio::Calibration()
{
    double reading[3] = {0,0,0};
    double noise_filtering_constant = 0.01;
    int i = 0;
    while(i<50)
    {
        MPU_.readGyro(reading); 
        if(fabs(reading[2]) < TOLERANCE)//Pegando só o do eixo  Z já que se joga em um campo plano , o que importa pra odometria é a rotação no seu própio eixo
        {
            i++;
        }
        calibration_result += reading[2] * noise_filtering_constant; // Amortiza o efeito de uma leitura irregular se tiver
    }
    std::cout << "resultado da calibração: "<< calibration_result << std::endl; 
}//Preferi fazer usando uma constante amortizadora pois faz com que uma leitura individual não tenha um peso muito grande no resultado final 

void Giroscopio::Update_flag()
{
    gyro_flag = true;
}

template <class ToDuration, class T = float>
T burocraciaTimer(const Timer& timer) 
{
    return chrono::duration_cast<chrono::duration<T, typename ToDuration::period>>(timer.elapsed_time()).count();
}//Tudo isso pra fazer o timer funcionar ...

void Giroscopio::Process_gyro() 
{
    
    if (gyro_flag && sensor_flag && !stop_flag) 
    {
        double reading[3] = {0,0,0};
        double outputReading = 0;//Output depois de diminuir o ruido com as samples 
        timer.start();
        for (int i = 0; i < numberSamples; i++) 
        {
            MPU_.readGyro(reading); 
            outputReading = reading[2] - calibration_result; 
        
        }
        outputReading /= numberSamples; 
        outputReading = outputReading * (M_PI / 180.0); //Formula para convertar para radianos
        if (fabs(outputReading) < 0.01) {
            outputReading = 0;
        }//Anulando leituras muito pequenas (irrelevantes)
        gyro_flag = false; 
        Gyro_speed = outputReading;
        std::cout << "Velocidade angular = " << Gyro_speed << " Rads/s" << std::endl;
        Gyro_angle = Gyro_speed * burocraciaTimer<chrono::milliseconds>(timer); //Pegando o deslocamento angular
        timer.reset(); 
        std::cout << "Deslocamento angular no pequeno intervalo = " << Gyro_angle << " Rads" << std::endl;
        Gyro_angle_cumulativo += Gyro_angle;
        if(Gyro_angle_cumulativo > M_PI)
        {
            Gyro_angle_cumulativo -= 2 * M_PI;
        }
        else if(Gyro_angle_cumulativo < -M_PI)
        {
            Gyro_angle_cumulativo += 2 * M_PI;
        }//Limitando o angulo do robo para dentro do intervalo -PI , +PI
        std::cout << "Posição do robo no seu eixo =  " << Gyro_angle_cumulativo << " Rads " << std::endl;
    }
    if(stop_flag)
    {
        std::cout << "Leitura do giroscopio finalizada" << std::endl;
        exit(1);
    }
}


int main(void)
{
    Giroscopio giro;
    giro.init();
    //Esse while 1 só começa quando o robo realmente tiver se movendo , quando o giroscopio já foi calibrado e a partida começou (em um caso real)
    while(1)
    {
        giro.Process_gyro();
    }
    return 0;
}


/*








































































































                                                                                                                                                                                                                   
                                                                                                                                                                                                                CODIGO SECRETO PARA LANÇAMENTO DAS BOMBAS NUCLEARES : 45135678









*/