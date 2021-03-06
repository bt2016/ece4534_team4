import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


# Bind the socket to the port
server_address = ('192.168.42.1', 10000)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

timer = 0

while True:
    # Wait for a connection
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()


    try:
        print >>sys.stderr, 'connection from', client_address

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(96)
            # hexdata = hex(data)
            # print >>sys.stderr, 'received "%s"' % data
            # print >>sys.stderr, '"%x"' % hexdata
            if "TIME" not in data:
                print >>sys.stderr, " ".join(hex(ord(n)) for n in data)
            else:
                timer = timer + 1
                print >>sys.stderr, '%f minutes' % timer
            
            """
            if data:
                print >>sys.stderr, 'sending data back to the client'
                connection.sendall(data)
            else:
                print >>sys.stderr, 'no more data from', client_address
                break
            """

    finally:
        # Clean up the connection
        print("closing the connection")
        connection.close()
