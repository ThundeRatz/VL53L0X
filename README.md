# VL53L0X

Esse repositório contém uma biblioteca para lidar com o sensor de distância [VL53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) da ST.

Essa biblioteca foi feita para ser utilizadas como submódulo no [STM32ProjectTemplate](https://github.com/ThundeRatz/STM32ProjectTemplate).

## Adicionando o submódulo ao projeto

Crie um diretório chamado `lib`, caso não exista:

```bash
mkdir lib
```
E adicione o submódulo fazendo:

* Com HTTPS:
```bash
git submodule add --name VL53L0X https://github.com/ThundeRatz/VL53L0X.git lib/VL53L0X
```

* Com SSH:
```bash
git submodule add --name VL53L0X git@github.com:ThundeRatz/VL53L0X.git lib/VL53L0X
```

## Utilizando a biblioteca

Para utilizar a biblioteca, é necessário que, para cada sensor utlizado, sejam criadas as seguintes variáveis.

```C
VL53L0X_Dev_t device;
VL53L0X_DeviceInfo_t device_info;
VL53L0X_RangingMeasurementData_t ranging_data;
vl53l0x_calibration_data_t calibration;
```

Cada sensor deve ser inciializado separadamente com a função:

```C
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t* p_device, VL53L0X_DeviceInfo_t device_info, vl53l0x_calibration_data_t calibration);
```

Quando utilizar mais de um sensor na mesma aplicação, é necessário desligar todos incialmente, e, individualmente, ligar, trocar o endereço e iniciar os sensores. Esse processo é feito com as funções abaixo, seguidas pela função de init.

```C
void vl53l0x_turn_off(VL53L0X_Dev_t* p_device);

VL53L0X_Error vl53l0x_wait_boot(VL53L0X_Dev_t* p_device);

VL53L0X_API VL53L0X_Error VL53L0X_SetDeviceAddress(VL53L0X_DEV Dev, uint8_t DeviceAddress);
```

É importante que o usuário verifique o valor retornado por todas as funções acima. Valores diferentes de 0 indicam má inicialização de algum sensor, o que compromete a incialização dos subsequentes e pode causar um comportamento estranho em todas as leituras.

E finalmente, para atualizar a leitura do sensor, utiliza-se a função:

```C
uint8_t vl53l0x_update_range(VL53L0X_Dev_t* p_device, VL53L0X_RangingMeasurementData_t* p_ranging_data,
                            uint16_t* range, uint16_t max_range);
```

Onde ```uint16_t* range``` armazena o valor da leitura.

Além disso, a seguinte função deve ser implementada pelo usuário. Ela deve implementar um delay que será usado na biblioteca.
```C
VL53L0X_Error VL53L0X_Delay(uint32_t ms);
```


---------------------

Equipe ThundeRatz de Robótica
