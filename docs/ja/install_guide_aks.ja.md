# RoboticBase Coreインストールガイド(Microsoft Azure AKS)

## 構築環境

||version|
|:--|:--|
|OS|macOS Mojave 10.14.6<br/>Ubuntu 16.04|
|pyenv|1.2.16|
|pipenv|2018.11.26|
|kubectl|1.17.3|
|helm|3.1.1|
|openssl|2.6.5|
|azure cli|2.1.0|

## 準備
<details><summary>`pyenv`と`pipenv`のインストール</summary>
<p>
### macOS

    ```
    $ brew install pyenv
    $ echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
    $ echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
    $ echo 'eval "$(pyenv init -)"' >> ~/.bashrc
    $ source ~/.bashrc
    $ brew install pipenv
    ```

### ubuntu

    ```
    $ sudo apt install -y build-essential libffi-dev libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev git python3-pip
    $ git clone https://github.com/pyenv/pyenv.git ~/.pyenv
    $ echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
    $ echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
    $ echo 'eval "$(pyenv init -)"' >> ~/.bashrc
    $ source ~/.bashrc
    $ pip3 install pipenv
    ```

</p>
</details>

<details><summary>`kubectl`のインストール</summary>
<p>
### macOS

    ```
    $ curl -LO "https://storage.googleapis.com/kubernetes-release/release/v1.17.3/bin/darwin/amd64/kubectl"
    $ chmod +x ./kubectl
    $ sudo mv ./kubectl /usr/local/bin/kubectl
    ```

### ubuntu

    ```
    $ curl -LO "https://storage.googleapis.com/kubernetes-release/release/v1.17.3/bin/linux/amd64/kubectl"
    $ chmod +x ./kubectl
    $ sudo mv ./kubectl /usr/local/bin/kubectl
    ```

</p>
</details>

<details><summary>`helm`のインストール</summary>
<p>
### macOS

    ```
    $ curl -LO "https://get.helm.sh/helm-v3.1.1-darwin-amd64.tar.gz"
    $ tar xvfz helm-v3.1.1-darwin-amd64.tar.gz
    $ sudo mv darwin-amd64/helm /usr/local/bin/helm
    ```

### ubuntu

    ```
    $ curl -LO "https://get.helm.sh/helm-v3.1.1-linux-amd64.tar.gz"
    $ tar xvfz helm-v3.1.1-linux-amd64.tar.gz
    $ sudo mv linux-amd64/helm /usr/local/bin/helm
    ```

</p>
</details>

<details><summary>`openssl`のインストール</summary>
<p>
### macOS

    ```
    $ brew install openssl
    ```

### ubuntu

    ```
    $ sudo apt install -y openssl
    ```

</p>
</details>

<details><summary>`minikube`のインストール</summary>
<p>
### macOS

    ```
    $ brew install azure-cli
    ```

### ubuntu

    ```
    $ curl -sL https://aka.ms/InstallAzureCLIDeb | sudo bash
    ```

</p>
</details>
