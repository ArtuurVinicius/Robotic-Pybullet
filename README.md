# Projeto Eletrônica Aplicada à Robótica
## Projeto robótica utilizando pybullet para criação de simulação

Integrantes:
 - Adrian Modesto Lauzid
 - Artur Vinícius Lima Ramos da Silva
 - Gustavo dos Santos Silva
 - Lucas Pereira de Souza

## Versões
O repositório contém duas branches para atender a diferentes necessidades de execução:

# main
Contém a versão padrão e simplificada do projeto.
Adequada para ambientes Linux ou versões do Windows que não possuem restrições com caminhos de diretório longos.
Não possui lógica extra para conversão de caminhos de arquivo.

# windows-compatibility-pathfix
Inclui uma função que converte caminhos longos para o formato curto (8.3) no Windows.
Feita devido a problemas de compatibilidade ao carregar arquivos de caminho longo no PyBullet.