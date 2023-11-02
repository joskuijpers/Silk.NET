FROM mcr.microsoft.com/dotnet/runtime:8.0 AS base
WORKDIR /app

FROM mcr.microsoft.com/dotnet/sdk:8.0 AS build
WORKDIR /src
COPY ["src/Lab/Experiments/BepuPhysicsDemo/BepuPhysicsDemo.csproj", "src/Lab/Experiments/BepuPhysicsDemo/"]
RUN dotnet restore "src/Lab/Experiments/BepuPhysicsDemo/BepuPhysicsDemo.csproj"
COPY . .
WORKDIR "/src/src/Lab/Experiments/BepuPhysicsDemo"
RUN dotnet build "BepuPhysicsDemo.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "BepuPhysicsDemo.csproj" -c Release -o /app/publish /p:UseAppHost=false

FROM base AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT ["dotnet", "BepuPhysicsDemo.dll"]
