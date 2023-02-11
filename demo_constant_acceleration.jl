using CSV
using DataFrames
using PyPlot

include("KalmanFilter.jl")

args = ARGS

file = "touchdata-20230210-111454_1.csv"
sigma_process = 0.01
sigma_measurement = 1

len = length(args)
if len > 2
	file = args[1]
	sigma_process = parse(Float64,args[2])
	sigma_measurement = parse(Float64,args[3])
elseif len > 1
	file = args[1]
	sigma_process = parse(Float64,args[2])
elseif len > 0
	file = args[1]
end

println(file)
println(sigma_process)
println(sigma_measurement)

df = CSV.File(file)|>DataFrame

dt = 1.0
kf = KalmanFilter(3, 1)
kf.F = [1.0 dt 0.5*dt*dt; 0.0 1.0 dt ; 0.0 0.0 1.0]
g = [0.5*dt*dt dt 1]'
kf.Q = g * g'
kf.Q = kf.Q * sigma_process
kf.H = [1.0 0.0 0.0]
kf.R = [sigma_measurement;;]

res = []

for i = 1 : size(df.vec)[1]
	if i == 1
		kf.x[1] = df.vec[1]
	else
		predict(kf)
		update(kf, [convert(Float64,df.vec[i]);;])
	end
	push!(res, kf.x[1])
end

plot(df.time, df.vec)
plot(df.time, res)

show()