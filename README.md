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

A seguinte função deve ser implementada pelo usuário. Ela deve implementar um delay que será usado na biblioteca
```C
VL53L0X_Error VL53L0X_Delay(uint32_t ms);
```


---------------------

Equipe ThundeRatz de Robótica
