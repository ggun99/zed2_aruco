sudo x11vnc \
> -display :2 \
> -auth /run/user/1000/gdm/Xauthority \
> -forever \
> -shared \
> -rfbport 5900 \
> -rfbauth /home/nvidia/.vnc/passwd
