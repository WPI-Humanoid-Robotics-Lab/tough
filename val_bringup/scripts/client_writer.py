#!/usr/bin/env python

import sys
import socket
import select

FIELD_IP = "192.168.2.10"
PORT = 8080


def broadcast_client():
    host = FIELD_IP
    port = PORT

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)

    # connect to remote host
    try:
        s.connect((host, port))
    except:
        print 'Unable to connect'
        sys.exit()

    print 'Connected to remote host. You can start sending messages'
    sys.stdout.write('[OCU] ');
    sys.stdout.flush()

    while 1:
        socket_list = [sys.stdin, s]

        # Get the list sockets which are readable
        ready_to_read, ready_to_write, in_error = select.select(socket_list, [], [])

        for sock in ready_to_read:
            if sock == s:
                # incoming message from remote server, s
                data = sock.recv(4096)
                if not data:
                    print '\nDisconnected from braodcast server'
                    sys.exit()
                else:
                    pass
                    # print data
#                    sys.stdout.write(data)
#                    sys.stdout.write('[OCU] ');
#                    sys.stdout.flush()

            else:
                # user entered a message
                msg = sys.stdin.readline()
                s.send(msg)
                sys.stdout.write('[OCU] ');
                sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(broadcast_client())
