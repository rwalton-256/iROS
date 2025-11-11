import socket
import iphone
import ctypes

def receive(sock, message_size):
     chunks = []
     bytes_recd = 0
     while bytes_recd < message_size:
          chunk = sock.recv(min(message_size - bytes_recd, 2048)) # Receive in chunks
          if not chunk:
               raise RuntimeError("Socket connection broken")
          chunks.append(chunk)
          bytes_recd += len(chunk)
     return b''.join(chunks)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
     s.bind(('0.0.0.0',8891))
     s.listen(5)
     client_sock,addr = s.accept()
     print(addr)

     ii = 0
     while True:
          header = iphone.Header.from_buffer_copy( receive( client_sock, ctypes.sizeof( iphone.Header ) ) )
          assert header.payload_length
          print(f"Image size: {header.payload_length} bytes")
          print(f"   {header.timestamp_sec+1e-9*header.timestamp_nsec}")
          data = receive( client_sock, header.payload_length )
          with open(f"{ii}.jpg",'wb') as f:
               f.write(data)
          ii += 1

