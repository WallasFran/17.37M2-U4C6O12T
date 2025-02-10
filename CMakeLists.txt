cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Inicialize pico_sdk a partir da localização instalada
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico CACHE STRING "Tipo de placa")

# Importar SDK do Raspberry Pi Pico
include(pico_sdk_import.cmake)

project(Leitura_serial_Display C CXX ASM)

# Inicialize o SDK do Raspberry Pi Pico
pico_sdk_init()

# Adicionar o executável. Alterando o nome do arquivo de DisplayC.c para Leitura_serial_Display.c
add_executable(Leitura_serial_Display Leitura_serial_Display.c inc/ssd1306.c)

# Definir nome e versão do programa
pico_set_program_name(Leitura_serial_Display "Leitura_serial_Display")
pico_set_program_version(Leitura_serial_Display "0.1")

# Configurar se o output via UART/USB está habilitado
pico_enable_stdio_uart(Leitura_serial_Display 0)
pico_enable_stdio_usb(Leitura_serial_Display 1)

# Adicionar a biblioteca padrão ao projeto
target_link_libraries(Leitura_serial_Display pico_stdlib)

# Incluir arquivos padrão
target_include_directories(Leitura_serial_Display PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}  # Diretório atual
)

# Adicionar a biblioteca hardware_i2c
target_link_libraries(Leitura_serial_Display hardware_i2c)

# Incluir diretório para arquivos específicos do hardware, incluindo o pio.h
target_include_directories(Leitura_serial_Display PRIVATE
  ${PICO_SDK_PATH}/src/rp2_common/hardware_pio/include  # Diretório do pio.h do SDK
)

# Adicionar a biblioteca hardware_pio
target_link_libraries(Leitura_serial_Display hardware_pio)

# Incluir o diretório que contém o arquivo ws2818b.pio.h
target_include_directories(Leitura_serial_Display PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/pio_files  # Altere o caminho conforme necessário
)

# Gerar o arquivo PIO para o controle do WS2818
pico_generate_pio_header(Leitura_serial_Display ${CMAKE_CURRENT_LIST_DIR}/ws2818b.pio)

# Adicionar saídas extras ao projeto
pico_add_extra_outputs(Leitura_serial_Display)
