
# Polynomial Neural network
class PolyNet(nn.Module):

	def __init__(self, in_channels=1, n_classes=10):
		super().__init__()
		N = 16
		kwds1 = { "kernel_size": 4, "stride": 2, "padding": 1 }
		kwds2 = { "kernel_size": 5, "stride": 1, "padding": 0 }
		kwds3 = { "kernel_size": 3, "stride": 1, "padding": 1 }

		self.conv11 = nn.Conv2d(in_channels, N, **kwds3)
		self.conv12 = nn.Conv2d(in_channels, N, **kwds3)

		self.conv21 = nn.Conv2d(N , N*2, **kwds1)
		self.conv22 = nn.Conv2d(N, N*2, **kwds1)

		self.conv31 = nn.Conv2d(N * 2, N * 4, **kwds1)
		self.conv32 = nn.Conv2d(N * 2 , N * 4, **kwds1)

		self.conv41 = nn.Conv2d(N * 4, N * 8, **kwds2)
		self.conv42 = nn.Conv2d(N * 4, N * 8, **kwds2)

		self.conv51 = nn.Conv2d(N * 8, N * 16, **kwds1)
		self.conv52 = nn.Conv2d(N * 8, N * 16, **kwds1)

		self.fc = nn.Linear(N * 16 * 3 * 3, n_classes)

	def forward(self, x):
		h = self.conv11(x) * self.conv12(x)
		h = self.conv21(h) * self.conv22(h)
		h = self.conv31(h) * self.conv32(h)
		h = self.conv41(h) * self.conv42(h)
		h = self.conv51(h) * self.conv52(h)
		h = self.fc(h.flatten(start_dim=1))

		return h

def main(args):
	log_dir_path = Path(args.o)
	try:
		log_dir_path.mkdir(parents=True)
	except FileExistsError:
		pass

	if torch.cuda.is_available():
		args.device = torch.device("cuda")
		print("GPU mode")
	else:
		args.device = torch.device("cpu")
		print("CPU mode")

	polynet = PolyNet(in_channels=(1 if args.d == "mnist" else 3)).to(args.device)

	kwds = {"root": ".", "download": True, "transform": transforms.ToTensor()}
	dataset_class = {"mnist": MNIST, "cifar10": CIFAR10}[args.d]
	train_dataset = dataset_class(train=True, **kwds)
	test_dataset = dataset_class(train=False, **kwds)
	train_loader = data.DataLoader(train_dataset, batch_size=args.b, shuffle=True)
	test_loader = data.DataLoader(test_dataset, batch_size=args.b)

	opt = torch.optim.Adam(net.paramaters(), lr=args.lr, betas=args.betas, weight_decay=args.weight_decay)
	trainer = create_supervised_trainer(net, opt, F.cross_entropy, device=args.weight_decay)

	metrics = {
		"accuracy" : Accuracy(),
		"loss" : Loss(F.cross_entropy)
	}

	evaluator = create_supervised_evaluator(net, metrics=metrics, device=args.device)
	trainer.add_event_handler(Events.EPOCH_COMPLETED, evaluate(evaluator, train_loader, test_loader, log_dir_path))

	if args.cg:
		trainer.add_event_handler(Events.ITERATION_STARTED(once=1), computational_graph(net, train_dataset, log_dir_path, device=args.device))
	trainer.run(train_loader, max_epoch=args.e)