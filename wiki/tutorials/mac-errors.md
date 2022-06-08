---
title: Mac Errors
date: 2022-06-07 00:00:00 Z
---

## Prerequisites
Please check whether you have completed all of the steps carefully for [ROS setup](ros) and that you have setup  [X-forwarding](x-forwarding) with xQuartz

## Unable to open display "host.docker.internal:0"

### Solution 0: Did you install everything?
0. Did you install homebrew?
    > [homebrew installation guide](https://docs.brew.sh/Installation)
1. Did you install docker?

    Download with Homebrew
    ```bash
    brew install docker
    ```
    It is also nice to install the Desktop app itself:
    > [Docker download](https://docs.docker.com/desktop/mac/install/) 

    You might want to make a docker account and select the free plan

2. Did you install xQuartz?

    Download with Homebrew:
    ```bash
    brew install --cask xquartz
    ```
    Download Online:
    >[xquartz download](https://www.xquartz.org/)
 
3. Did you install xeyes?

    xeyes is command that can test whether you have configured your environment. 

    Note: You need to install macports to run it (see below).
    ```bash
    sudo port install xeyes
    ```
    Run the following command to see a pair of eyes following your cursor! 
    ```bash
    xeyes
    ```
    If it does not work, it means that your setup is broken.

4. (Optional) Did you install macports?

    >Macport installation guide: [Macport guide](https://www.macports.org/install.php)

### Solution 1: Did you configure xQuartz, xhost and Docker?
1. Check if you  installed xquartz in solution 0

2. open xquartz through terminal with 
    ```bash
    open -a XQuartz
    ```
3. Go to x11 preferences and security, and make both checkmarks are selected:
    ![XQuartz Setting](assets/images/xquartz-setting.png)

4. Restart your Mac 
    ```bash
    sudo shutdown -r now
    ```
5. After restarting, start up xQuartz with
    ```bash
    open -a XQuartz
    ```
6. Start xhost with
    ```bash
    xhost
    ```
7. Continue with the guide!

### Solution 2: Did you set your Display IP address?
1. Attach your local ip address ```127.0.0.1``` to xhost:
    ```bash
    xhost + YOUR_IP_ADDRESS_FROM_ABOVE
    ```

If this doesnt work:

1. Get your local ip address with 
    ```bash
    ifconfig en0 | grep inet | awk '$1=="inet" {print $2}
    ```
2. Attach the result to xhost:
    ```bash
    xhost + YOUR_IP_ADDRESS_FROM_ABOVE
    ```
3. Continue with the guide!

### Solution 3: Manually set the display: 

1. Set the display with
    ```bash
    DISPLAY=127.0.0.1:0
    ```
2. initialize xhost:
    ```bash
    xhost
    ```
3. open xhost ports:
    ```bash
    xhost +
    ```
4. try steps 1 to 3 with your ip address from Solution 2
    Use the following command in step 1:
    ```bash
    DISPLAY=YOUR_IP_ADDRESS:0
    ```
5. Continue to follow the guide!

### Solution 4: Try combination of Solution 2 and 3:
1. Use the commands in a different order, after starting xquartz and docker
    ![xhost debug example](assets/images/xhost-debug-example.png)

### Solution 5: add x11 server through Mac Ports:
1. If you don't have ```macport``` installed:
    >Macport installation guide: [Macport guide](https://www.macports.org/install.php)
2. Install x11 server through:
    ```bash
    sudo port install xorg-server
    ```
3. Continue to follow the guide or try Solutions 1-4!

### Helpful Links:

>[Medium Article Guide](https://medium.com/crowdbotics/a-complete-one-by-one-guide-to-install-docker-on-your-mac-os-using-homebrew-e818eb4cfc3)

>[Oracle Archives Guide](https://web.archive.org/web/20190514151847/https://blogs.oracle.com/oraclewebcentersuite/running-gui-applications-on-native-docker-containers-for-mac)

>[Legendary Stack Overflow Answer #1](https://stackoverflow.com/questions/38686932/how-to-forward-docker-for-mac-to-x11)
    
>[Legendary Stack Overflow Answer #2](https://stackoverflow.com/questions/28392949/running-chromium-inside-docker-gtk-cannot-open-display-0/34586732#comment63471630_28395350)
