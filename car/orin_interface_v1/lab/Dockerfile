ARG BASE_IMAGE=ultralytics/ultralytics@sha256:e47a232c790dd65bc0b51a50057b5f05fe4d8edfc532a184384045166a44f5ef
FROM ${BASE_IMAGE}
RUN python3 -m pip install --no-cache-dir 'jupyterlab==4.4.9'
ENV HOME=/tmp PYTHONUNBUFFERED=1
WORKDIR /workspace
RUN chmod a+r /ultralytics/yolo26n.pt
CMD ["python3", "-m", "jupyterlab", "--config=/etc/jupyter/jupyter_server_config.py"]
