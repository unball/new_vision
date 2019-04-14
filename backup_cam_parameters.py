#!/usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
#      @description: Get actual parameters from file that represent the cam and store it 
#      in a file using v4l2-ctl program or get the informations in backup file and copy to 
#      file that represent the cam in system

from subprocess import check_call

CAM_INDEX = '1'
BACKUP_FILE = 'cam_config.txt'
GET_PROPERTIES_COMMAND = 'v4l2-ctl -d /dev/video' + CAM_INDEX + ' --list-ctrls'
SET_PROPERTIES_COMMAND = 'v4l2-ctl -d /dev/video' + CAM_INDEX + ' --set-ctrl='
MENU = (('Opções da câmera de índice%s:'%(CAM_INDEX)),
        ('> 1 - Criar backup de configuração'),
        ('> 2 - Usar configurações do backup\n')) 

class Cam(object):
    def __init__(self):
        self.index_automatic_parameters = []
        self.parameters = [] # example: [['brightness', '0', '(int)'], 
                             #          ['contrast', '0', '(int)'], 
                             #          ['focus_auto','0', '(bool)']]
    
    def find_index_automatic_parameters(self):
        for parameter in range(len(self.parameters)):
            type_parameter = self.parameters[parameter][2]
            if type_parameter == '(bool)':
                index_parameter_auto = self.parameters[parameter][0].find('auto')
                if not index_parameter_auto == -1 :
                    self.index_automatic_parameters.append(parameter)

def file_exist_and_can_open():
    """
    Return 0 if don't happen any error
    """
    try:
        with open(BACKUP_FILE) as f:
            return 1
    except:
        return 0

def get_config():
    """
    Return 1 if don't happen any error
    """
    return not check_call(GET_PROPERTIES_COMMAND + '> ' + BACKUP_FILE, shell=True)

def import_parameters_from_file(backup):
    """
    Return 1 if don't happen any error
    """
    try:
        with open(BACKUP_FILE) as f:
            file = f.readlines()
            for x in range(len(file)):
                line = file[x]
                if not line.find('flags=inactive') == -1:
                    continue

                name_parameter = line.split()[0]
                type_parameter = line.split()[1]

                init_index_value_parameter = line.find('value=') + len('value=')
                final_index_value_parameter = line.find(' ', init_index_value_parameter)
                if final_index_value_parameter == -1:
                    final_index_value_parameter = len(line) - 1

                value_parameter = line[init_index_value_parameter:final_index_value_parameter]

                backup[0].parameters.append([name_parameter, value_parameter, type_parameter])
        return 1
    except:
        return 0

def insert_parameters_in_cam(backup):
    """
    Return 1 if don't happen any error
    """
    backup[0].index_automatic_parameters.sort(reverse=True)
    try:
        for index in backup[0].index_automatic_parameters:
            # print(SET_PROPERTIES_COMMAND + backup[0].parameters[index][0] + '=0')
            check_call(SET_PROPERTIES_COMMAND + backup[0].parameters[index][0] + '=0', shell=True)
            del backup[0].parameters[index]
        for parameter in backup[0].parameters:
            # print(SET_PROPERTIES_COMMAND + parameter[0] + '=' + parameter[1])
            check_call(SET_PROPERTIES_COMMAND + parameter[0] + '=' + parameter[1], shell=True)
        return 1
    except:
        return 0

if __name__ == "__main__":    
    for x in MENU:
        print(x)
    choice = input('>> Escolha: ')
    backup = [Cam()]

    if choice == '1':
        if get_config():
            print("Backup criado com sucesso!")
        else:
            print("Não foi possível obter as configurações da câmera de índice%s \
                e armazena-las no arquivo \"%s\"" %(CAM_INDEX, BACKUP_FILE))
    elif choice == '2':
        if not file_exist_and_can_open():    
            print("O arquivo \"%s\" de configuração não pode ser aberto ou não existe."%BACKUP_FILE)
        elif import_parameters_from_file(backup):
            backup[0].find_index_automatic_parameters()
            if not insert_parameters_in_cam(backup):
                print("Ocorreu algum erro em passar os parâmetros do arquivo de backup para o arquivo da câmera")
            else:
                print("Configurações resetadas com sucesso!")
    else:
        print("Escolha incorreta!")
