param(
    [string]$ListenAddress = "0.0.0.0",
    [int]$ListenPort = 8888,
    [string]$AgentAddress = "127.0.0.1",
    [int]$AgentPort = 8888
)

$front = $null
$back = $null

try {
    $frontEndpoint = [System.Net.IPEndPoint]::new(
        [System.Net.IPAddress]::Parse($ListenAddress),
        $ListenPort)
    $agentEndpoint = [System.Net.IPEndPoint]::new(
        [System.Net.IPAddress]::Parse($AgentAddress),
        $AgentPort)

    $front = [System.Net.Sockets.UdpClient]::new($frontEndpoint)
    $back = [System.Net.Sockets.UdpClient]::new(
        [System.Net.Sockets.AddressFamily]::InterNetwork)

    $clientEndpoint = $null
    Write-Output "relay ready: ${ListenAddress}:${ListenPort} -> ${AgentAddress}:${AgentPort}"

    while ($true) {
        if ($front.Available -gt 0) {
            $remote = [System.Net.IPEndPoint]::new(
                [System.Net.IPAddress]::Any,
                0)
            $payload = $front.Receive([ref]$remote)
            $clientEndpoint = $remote
            [void]$back.Send($payload, $payload.Length, $agentEndpoint)
            Write-Output "client->agent bytes=$($payload.Length) client=$remote"
        }

        if ($back.Available -gt 0) {
            $remote = [System.Net.IPEndPoint]::new(
                [System.Net.IPAddress]::Any,
                0)
            $payload = $back.Receive([ref]$remote)
            if ($null -ne $clientEndpoint) {
                [void]$front.Send($payload, $payload.Length, $clientEndpoint)
                Write-Output "agent->client bytes=$($payload.Length) client=$clientEndpoint"
            }
        }

        Start-Sleep -Milliseconds 1
    }
}
finally {
    if ($null -ne $back) {
        $back.Dispose()
    }
    if ($null -ne $front) {
        $front.Dispose()
    }
}
