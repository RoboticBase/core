# RoboticBase Coreインストールガイド(minikube)

## 構築環境

||version|
|:--|:--|
|OS|macOS Mojave 10.14.6<br/>Ubuntu 16.04|
|pyenv|1.2.16|
|pipenv|2018.11.26|
|kubectl|1.17.3|
|helm|3.1.1|
|VirtualBox|6.1.2 r135662|
|minikube|1.7.3|

## 準備
### ツールのインストール
<details><summary><b>pyenv</b>と<b>pipenv</b>のインストール</summary>
<p>

#### macOS

```
$ brew install pyenv
$ echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
$ echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
$ echo 'eval "$(pyenv init -)"' >> ~/.bashrc
$ source ~/.bashrc
$ brew install pipenv
```

#### ubuntu

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

<details><summary><b>kubectl</b>のインストール</summary>
<p>

#### macOS

```
$ curl -LO "https://storage.googleapis.com/kubernetes-release/release/v1.17.3/bin/darwin/amd64/kubectl"
$ chmod +x ./kubectl
$ sudo mv ./kubectl /usr/local/bin/kubectl
```

#### ubuntu

```
$ curl -LO "https://storage.googleapis.com/kubernetes-release/release/v1.17.3/bin/linux/amd64/kubectl"
$ chmod +x ./kubectl
$ sudo mv ./kubectl /usr/local/bin/kubectl
```

</p>
</details>

<details><summary><b>helm</b>のインストール</summary>
<p>

#### macOS

```
$ curl -LO "https://get.helm.sh/helm-v3.1.1-darwin-amd64.tar.gz"
$ tar xvfz helm-v3.1.1-darwin-amd64.tar.gz
$ sudo mv darwin-amd64/helm /usr/local/bin/helm
```

#### ubuntu

```
$ curl -LO "https://get.helm.sh/helm-v3.1.1-linux-amd64.tar.gz"
$ tar xvfz helm-v3.1.1-linux-amd64.tar.gz
$ sudo mv linux-amd64/helm /usr/local/bin/helm
```

</p>
</details>

<details><summary><b>Oracle VM VirtualBox</b>のインストール</summary>
<p>

#### macOS

```
$ curl -Lo ~/Downloads/VirtualBox-6.1.2-135662-OSX.dmg https://download.virtualbox.org/virtualbox/6.1.2/VirtualBox-6.1.2-135662-OSX.dmg
$ open ~/Downloads/VirtualBox-6.1.2-135662-OSX.dmg
$ open /Applications/VirtualBox.app
```

#### Ubuntu

```
$ wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -
$ wget -q https://www.virtualbox.org/download/oracle_vbox.asc -O- | sudo apt-key add -
$ sudo add-apt-repository "deb http://download.virtualbox.org/virtualbox/debian xenial contrib"
$ sudo apt-get update
$ sudo apt-get install -y virtualbox-6.1
```

</p>
</details>

<details><summary><b>minikube</b>のインストール</summary>
<p>

#### macOS

```
$ curl -Lo minikube https://storage.googleapis.com/minikube/releases/v1.7.3/minikube-darwin-amd64
$ chmod +x ./minikube
$ sudo mv ./minikube /usr/local/bin/minikube
```

#### Ubuntu

```
$ curl -Lo minikube https://storage.googleapis.com/minikube/releases/v1.7.3/minikube-linux-amd64
$ chmod +x ./minikube
$ sudo mv ./minikube /usr/local/bin/minikube
```

</p>
</details>

### ansibleの準備
<details><summary>RoboticBase/coreをclone</summary>
<p>

```
$ git clone https://github.com/RoboticBase/core.git
$ cd core
```

</p>
</details>

<details><summary><b>ansible</b>と関連するPythonライブラリのインストール</summary>
<p>

```
$ cd ansible
$ pipenv install
```

</p>
</details>

## RoboticBase/coreのインストール
### 変数の変更
1. 次のYAMLファイルに定義されているMQTTユーザー（`iotagent`）のパスワード(**mqtt.users[?name==\`iotagent\`].password**)を変更する
    * [group\_vars/all.yml](../../ansible/group_vars/all.yml)
1. 起動する各コンテナのレプリカ数等を変更したい場合には、次のYAMLファイルに定義されている値を変更する
    * [inventories/minikube/group\_vars/minikube.yml](../../ansible/inventories/minikube/group_vars/minikube.yml)
1. minikubeに与えるCPUやメモリ等を変更したい場合には、次のYAMLファイルに定義されている値を変更する
    * [inventories/minikube/host\_vars/localhost.yml](../../ansible/inventories/minikube/host_vars/localhost.yml)

### RoboticBase/coreを起動
1. pipenv shellを起動する

    ```
    $ pipenv shell
    ```
1. ansibleを用いてRoboticBase/coreをminikube上に起動する

    ```
    $ ansible-playbook -i inventories/minikube --extra-vars="ansible_python_interpreter=$(which python)" minikube.yml
    ```

### grafanaの設定
1. grafanaのServiceへ3000ポートをPort-Forwardする

    ```
    $ kubectl -n monitoring port-forward svc/po-grafana 3000:80
    ```
1. ブラウザでgrafana ([http://localhost:3000](http://localhost:3000))にアクセス
    ![grafana\_01.png](../images/minikube/grafana_01.png)
1. **email or username**に"admin"、**password**に"prom-operator"を入力し**Log In**する
    ![grafana\_02.png](../images/minikube/grafana_02.png)
1. 左下の**Preferences**より**Change Password**を選択し、adminのパスワードを変更する
    ![grafana\_03.png](../images/minikube/grafana_03.png)
1. ホーム画面より、minikubeの各種リソースを監視するダッシュボードがインストールされていることを確認する
    ![grafana\_04.png](../images/minikube/grafana_04.png)
1. Ctrl-Cでport-forwardingを終了する

### kibanaの設定
1. kibanaのServiceへ5601ポートをPort-forwardする

    ```
    $ kubectl -n logging port-forward svc/kibana 5601:80
    ```
1. ブラウザでkibana ([http://localhost:5601](http://localhost:5601))にアクセス
    ![kibana\_01.png](../images/minikube/kibana_01.png)
1. **Explore on my own**をクリックしてホーム画面を表示する
    ![kibana\_02.png](../images/minikube/kibana_02.png)
1. **Management**をクリックして管理画面を表示する
    ![kibana\_03.png](../images/minikube/kibana_03.png)
1. **Index Patterns**をクリックする
    ![kibana\_04.png](../images/minikube/kibana_04.png)
1. **Create Index Patterns**をクリックする
    ![kibana\_05.png](../images/minikube/kibana_05.png)
1. **Index pattern**に"logstash-\*"と入力し、**Next Step**をクリックする
    ![kibana\_06.png](../images/minikube/kibana_06.png)
1. **Time Filter field name**として"@timestamp"を選択し、**Create Index pattern**をクリックする
    ![kibana\_07.png](../images/minikube/kibana_07.png)
1. Indexが作成される
    ![kibana\_08.png](../images/minikube/kibana_08.png)
1. **Discover**をクリックし、Podのログが収集されていることを確認する
    ![kibana\_09.png](../images/minikube/kibana_09.png)
1. Ctrl-Cでport-forwardingを終了する
