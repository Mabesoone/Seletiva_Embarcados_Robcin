//CODIGO DA ENTRAGA PARCIAL DE EDUARDO MABESOONE MELO EMM4
//Dado que o input para o desafio é por um arquivo CSV e não pelo sensor , as partes que são para ler pelo sensor estou lendo do arquivo CSV
//Mas nessas partes eu comentei o codigo que funcionaria para ler do sensor(MPU6050), pelo datapath usual 

#include <iostream>
//#include <MPU6050/MPU6050.h>
//#include "mbed.h"
#include <fstream>
#include <cmath>
#include <vector>
#include <sstream>
#define TOLERANCE 0.5



class Giroscopio 
{
    public:
        //Giroscopio(); consturtor para inicializar o giroscopio, com os pins do data e do clock porem não necessarios para ler csv
        void init(const std::string& csv_path);
        int count_linhas = 0; //para saber quantas linhas se passaram no csv , para poder "skipar" as linhas até o frame certo onde o robo começa a se mexer 
        double calibration_result = 0; //Não é um vetor já que só é necessario ler do eixo Z (campo é sem disnível)
    private:
        //MPU6050 MPU_; MPU NÃO NECESSARIA PARA LER CSV ,POREM PARA LER OS DADOS DIRETO DO SENSOR RETIRA-SE ESSE COMENTÁRIO 
        void Calibration(); //Método para calibrar o giroscopio , para identificar  se a leitura está enviesada(offsets) para que as leituras futuras não estejam enviesadas
        void Update_flag(); //Método para dar update na flag do giroscopio , a flag determina se o giroscopio deve ser lido ou não
        void Process_gyro(); //Método para processar os dados do giroscopio e converter essa leitura para o formato correto (RAD/S , RAD)
        void LoadCSV(const std::string& csv_path); // Carregar os dados do CSV , nesse caso especifico dos samples
        bool stop_flag = false;//Flag pra fazer o programa parar de rodar , na pratica nao se usaria ela , mas só pra não ficar rodando o codigo por muito tempo 
        bool gyro_flag = false; //Flag para ver se o giroscopio precisa ser analisado
        bool IMU_flag = false; //Flag para ver se a IMU foi inicializada corretamente , nao necessária para ler o csv
        int numberSamples = 5; //Quantidade de samples usadas por vez para garatir o minimo de ruido possivel sem comprometer o tempo de resposta
        int flag_timer = 5;
        int process_timer = 1; //Os 2 em milisegundos
        double Gyro_angle = 0;
        double Gyro_speed = 0; // Resultados finais 
        double total = 0;
        

        /*comentados por enquanto
    
        //Ticker Flag_update; Ticker para chamar periodicamente o metodo de dar update na flag do gyro
        //Ticker Gyro_process; Ticker para chamar periodicamente o metodo de processar o 
        */

        

        std::vector<double> gyro_readings; // pra guardar a leitura do csv
};

void Giroscopio::init(const std::string& csv_path) // : MPU_(MPU_SDA, MPU_SCL) para pegar os fios pelo construtor , para ler do sensor 
{
    LoadCSV(csv_path);
    //IMU_flag = MPU_.initialize(); NAO NECESSARIO INICIALIZAR MPU PARA LER O CSV
    IMU_flag = true;//so para entrar
    if(IMU_flag)
    {
        //If everything is in order, the calibration can begin 
        Calibration();
        for( ;count_linhas<3675;count_linhas++)
        {
            gyro_readings.erase(gyro_readings.begin()); //apagando as linhas depois da calibraçao para pegar o só quando o robo esta em funciamento
        }
        // a calibração poderia ser feita até o final desse tempo porem achei que já que é preciso fazer outros procedimentos no robo sem ser a calibração do girscopio nesse meio tempo preferi manter uma quantidade menor de frames para a calibracao

        //First we calibrate the sensor when the robot is not moving so that aftwards we can use the offset for future readings

        //não esta pegando no meu pc o mbed.h portanto vou deixar comentado essas linhas para repeticao periodica que usa a classe ticker

        /* 
        Flag_update.attach(callback(this, &Giroscopio::Update_flag), chrono::milliseconds(flag_timer));
        Gyro_process.attach(callback(this, &Giroscopio::Process_gyro), chrono::milliseconds(process_timer));
        */
       gyro_flag = true;
       Process_gyro();

        //After the calibration is done , both flagupdate and process methods are schdulle to run in the predetermined timer 
    }
    else 
    {
         std::cout << "ERROR , Failure when initializing the Sensor";
         exit(1);
         //If an error ocurred and the MPU couldnt be initialized
    }
}

void Giroscopio::Calibration()
{
    //double reading[3] = {0,0,0};
    double reading;
    double noise_filtering_constant = 0.01;
    int i = 0;
    while(i<50)
    {
        count_linhas++;
        //reading = MPU_.readGyro(reading); USE reading[2] for z axis
        reading = gyro_readings.front();
        gyro_readings.erase(gyro_readings.begin()); 
        if(fabs(reading) < TOLERANCE)
        {
            i++;
        }
        calibration_result += reading * noise_filtering_constant;
        
    }
    std::cout << "resultado da calibração: "<< calibration_result << std::endl;
}

void Giroscopio::Update_flag()
{
    gyro_flag = true;
}


void Giroscopio::Process_gyro() 
{
    
    if (gyro_flag && IMU_flag) 
    {
        
         //reading = MPU_.readGyro(reading); USE reading[2] for z axis
        double reading = 0;
        while(!gyro_readings.empty() && gyro_readings.size()>=5)
        {
            for (int i = 0; i < numberSamples; i++) 
            {
                if (!gyro_readings.empty()) 
                {
                    reading += gyro_readings.front() - calibration_result; 
                    gyro_readings.erase(gyro_readings.begin()); 
                } 
                else 
                {
                    stop_flag = true;
                    break;
                }
            }
            if (!stop_flag) 
            {
                reading /= numberSamples; 
                reading = reading * (M_PI / 180.0); // Formula para convertar para radianos
                if (fabs(reading) < 0.01) {
                    reading = 0;
                }
                double tempo = 1.0 / 40;//Se passaram 5 samples portanto 1/200 * 5 Agora divide pelo tempo passado , numa frequencia de 0,2 khz 1 repitaçao demora 1/200 segundo
                
                gyro_flag = false; //linha para se tiver lendo do sensor real

                Gyro_speed = reading;
                std::cout << "Velocidade angular = " << Gyro_speed << " Rads/s" << std::endl;
                Gyro_angle = Gyro_speed * tempo;
                std::cout << "Deslocamento angular = " << Gyro_angle << " Rads" << std::endl;
                total += Gyro_angle;
                if(total > M_PI)
                {
                    total -= 2 * M_PI;
                }
                else if(total < -M_PI)
                {
                    total += 2 * M_PI;

                }
                std::cout << "Angulo cumulativo" << total << std::endl;
        }
    }
}
}

void Giroscopio::LoadCSV(const std::string& csv_path) 
{
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cout << "Error: Unable to open CSV file: " << csv_path << std::endl;
        exit(1);
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double first_number;
        if (ss >> first_number) 
        {
            gyro_readings.push_back(first_number); 
        } 
    }
    file.close();
}


int main(void)
{
    Giroscopio giro;
    giro.init("/Users/emabesoone/Downloads/filtered_robot_log-2.csv");
    std::cout << giro.count_linhas << std::endl; 
    std::cout << giro.calibration_result << std::endl;

    
    return 0;
}



//ate o frame 3675 o robo esta parado , a calibração continua do mesmo tamanho porem ignoro o resto do frame em que o robo esta parado
