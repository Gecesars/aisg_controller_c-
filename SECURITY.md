# Segurança da referência `ret_aisg2`

O clone analisado do `ret_aisg2` deve ser tratado como não confiável.

No commit `2b669c33318a1fe2dfc57a1ee265eccb6bbfbab3`, o arquivo `main.py` tem SHA-256
`1d4783e794df54ed49a98656ea908eb785a1521a5b97037c9c84d50e444938f3` e contém,
nas linhas 1240–1248, um payload ofuscado terminado por `exec(compile(...))`.
A análise estática e inerte identificou comportamento de downloader/dropper,
incluindo consulta de dados externos, download de Node.js, gravação de JavaScript
e execução de um segundo estágio.

Regras adotadas neste projeto:

- não executar, importar ou instalar módulos do clone;
- não usar `main.py` em nenhuma circunstância;
- consumir somente capturas de texto como entrada não executável;
- validar cada vetor de protocolo de forma independente;
- não copiar binários, firmware, especificações ou arquivos CAD;
- manter o novo aplicativo sem dependência de runtime do clone.

A busca pelos mesmos indicadores nos arquivos Python e ZIPs encontrou o padrão
apenas no `main.py`, mas isso não torna os demais arquivos confiáveis.
