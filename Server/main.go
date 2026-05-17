 
package main

import (
        "crypto/rand"
        "encoding/hex"
        "fmt"
        "net"
        "strings"
        "sync"
)

// Client описує підключеного користувача
type Client struct {
        ID         string
        TCPConn    net.Conn
        UDPAddr    *net.UDPAddr
        WatchingID string
}

// Server зберігає стан усіх підключень
type Server struct {
        clients map[string]*Client
        mutex   sync.RWMutex
        udpConn *net.UDPConn
}

func main() {
        srv := &Server{
                clients: make(map[string]*Client),
        }

        go srv.startUDP(":80")
        srv.startTCP(":80")
}

func (s *Server) startTCP(port string) {
        ln, err := net.Listen("tcp", port)
        if err != nil {
                fmt.Println("Error TCP:", err)
                return
        }
        defer ln.Close()
        fmt.Println("TCP Server on port", port)

        for {
                conn, err := ln.Accept()
                if err != nil {
                        continue
                }
                go s.handleTCPConnection(conn)
        }
}

func (s *Server) handleTCPConnection(conn net.Conn) {
        defer conn.Close()

        idBytes := make([]byte, 8)
        rand.Read(idBytes)
        clientID := hex.EncodeToString(idBytes)

        client := &Client{
                ID:      clientID,
                TCPConn: conn,
        }

        s.mutex.Lock()
        s.clients[clientID] = client
        s.mutex.Unlock()

        fmt.Printf("[TCP] New connection: %s\n", clientID)
        conn.Write([]byte(fmt.Sprintf("AUTH_OK:%s\n", clientID)))

        buffer := make([]byte, 1024)
        for {
                n, err := conn.Read(buffer)
                if err != nil {
                        fmt.Printf("[TCP] Disconnecting: %s\n", clientID)
                        s.mutex.Lock()
                        delete(s.clients, clientID)
                        s.mutex.Unlock()
                        return
                }

                command := strings.TrimSpace(string(buffer[:n]))
                s.handleCommand(client, command) // Виклик функції з handlers.go
        }
}

func (s *Server) startUDP(port string) {
        addr, _ := net.ResolveUDPAddr("udp", port)
        conn, err := net.ListenUDP("udp", addr)
        if err != nil {
                fmt.Println("Error UDP:", err)
                return
        }
        s.udpConn = conn
        defer conn.Close()
        fmt.Println("UDP Serev on port", port)

        buffer := make([]byte, 65535)

        for {
                n, remoteAddr, err := conn.ReadFromUDP(buffer)
                if err != nil || n <= 16 {
                        continue
                }

                senderID := string(buffer[:16])
                payload := buffer[16:n]

                s.mutex.RLock()
                sender, exists := s.clients[senderID]
                s.mutex.RUnlock()

                if !exists {
                        continue
                }

                if sender.UDPAddr == nil || sender.UDPAddr.String() != remoteAddr.String() {
                        sender.UDPAddr = remoteAddr
                }

                strPayload := string(payload)
                if strPayload == "HELLO" || strPayload == "PING" {
                        continue
                }


                s.routeUDPData(senderID, payload) // Виклик функції з handlers.go
        }
}